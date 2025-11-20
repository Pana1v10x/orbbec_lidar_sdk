// macp see https://github.com/foxglove/mcap/tree/main/cpp
#define MCAP_IMPLEMENTATION
#include "recorder.hpp"

#include <limits>
#include <mcap/writer.hpp>
#include <random>

#include "../logger.hpp"
#include "foxglove/PointCloud.pb.h"
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace foxglove {

/// Builds a FileDescriptorSet of this descriptor and all transitive
/// dependencies, for use as a channel schema.
google::protobuf::FileDescriptorSet BuildFileDescriptorSet(
    const google::protobuf::Descriptor* toplevelDescriptor) {
    google::protobuf::FileDescriptorSet fdSet;
    std::queue<const google::protobuf::FileDescriptor*> toAdd;
    toAdd.push(toplevelDescriptor->file());
    std::unordered_set<std::string> seenDependencies;
    while (!toAdd.empty()) {
        const google::protobuf::FileDescriptor* next = toAdd.front();
        toAdd.pop();
        next->CopyTo(fdSet.add_file());
        for (int i = 0; i < next->dependency_count(); ++i) {
            const auto& dep = next->dependency(i);
            if (seenDependencies.find(dep->name()) == seenDependencies.end()) {
                seenDependencies.insert(dep->name());
                toAdd.push(dep);
            }
        }
    }
    return fdSet;
}

}  // namespace foxglove

namespace ob_lidar::detail {

struct PointXYZI {
    float x;
    float y;
    float z;
    float intensity;
};

double deg2rad(double degree) { return degree * M_PI / 180.0; }

std::vector<PointXYZI> scanToPoints(const ob_lidar::LidarScan& scan) {
    std::vector<PointXYZI> points;
    points.reserve(scan.ranges.size());
    auto start_angle = scan.start_angle;
    auto angle_resolution = scan.angle_resolution;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        if (scan.ranges[i] <= 0) continue;  // Skip invalid measurements

        const auto angle_deg = start_angle + i * angle_resolution;
        double angle_rad = deg2rad(angle_deg);

        // note: here change the coordinate system direction, make it more
        // visual
        double x = scan.ranges[i] * cos(angle_rad) / 1000.0;
        double y = scan.ranges[i] * sin(angle_rad) / 1000.0;
        double z = 0.0;

        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            PointXYZI point{};
            point.x = x;
            point.y = y;
            point.z = z;
            point.intensity = static_cast<float>(scan.intensities[i]);
            points.push_back(point);
        }
    }
    return points;
}

RecorderImpl::RecorderImpl(std::shared_ptr<Device> device,
                           const std::string& output_dir)
    : device_(device), output_dir_(output_dir) {
    point_cloud_writer_ = std::make_shared<mcap::McapWriter>();
    auto options = mcap::McapWriterOptions("");
    const auto res = point_cloud_writer_->open(output_dir, options);
    if (!res.ok()) {
        LOG_ERROR("Failed to open mcap writer: {}", res.message);
        throw std::runtime_error("Failed to open mcap writer");
    }
    mcap::Schema schema(
        "foxglove.PointCloud", "protobuf",
        foxglove::BuildFileDescriptorSet(foxglove::PointCloud::descriptor())
            .SerializeAsString());
    point_cloud_writer_->addSchema(schema);
    point_cloud_writer_->addSchema(schema);
    mcap::Channel channel("pointcloud", "protobuf", schema.id);
    point_cloud_writer_->addChannel(channel);
    point_cloud_channel_id_ = channel.id;
    observer_id_ = generateObserverId();
    LOG_INFO("recorder output dir: {}", output_dir_);
    LOG_INFO("recorder observer id: {}", observer_id_);
}

RecorderImpl::~RecorderImpl() {
    enable_record_ = false;
    cv_.notify_one();
    stop();
    if (record_thread_ != nullptr && record_thread_->joinable()) {
        record_thread_->join();
        record_thread_.reset();
    }
    point_cloud_writer_->close();
}

void RecorderImpl::start() {
    device_->registerFrameObserver(
        observer_id_,
        [this](std::shared_ptr<Frame> frame) { frameCallback(frame); });
    if (record_thread_ != nullptr) {
        LOG_WARN("recorder thread already started");
        return;
    }
    enable_record_ = true;
    record_thread_ =
        std::make_shared<std::thread>(&RecorderImpl::recordThread, this);
}

void RecorderImpl::stop() {
    enable_record_ = false;
    cv_.notify_one();
    if (record_thread_ != nullptr && record_thread_->joinable()) {
        record_thread_->join();
        record_thread_.reset();
    }
    device_->unregisterFrameObserver(observer_id_);
    // clear queue
    std::unique_lock<std::mutex> lock(mutex_);
    while (!frame_queue_.empty()) {
        frame_queue_.pop();
    }
}

void RecorderImpl::frameCallback(std::shared_ptr<Frame> frame) {
    std::unique_lock<std::mutex> lock(mutex_);
    frame_queue_.push(frame);
    cv_.notify_one();
}

void RecorderImpl::recordFrame(const std::shared_ptr<Frame> frame) const {
    if (frame == nullptr || frame->isNull()) {
        LOG_ERROR("Received a nullptr frame.");
        return;
    }
    if (!enable_record_) {
        LOG_WARN("recorder is not enable");
        return;
    }
    // TODO: support imu and other frame type
    if (frame->type() == LidarFrameType::SCAN) {
        const auto scan_frame = std::dynamic_pointer_cast<ScanFrame>(frame);
        if (scan_frame == nullptr) {
            std::cerr << "Failed to cast frame to ScanFrame." << std::endl;
            return;
        }

        auto scan = scan_frame->toScan();
        const auto points = scanToPoints(scan);
        if (points.empty()) {
            return;
        }
        foxglove::PointCloud pcl;
        pcl.set_point_stride(sizeof(float) * 4);
        constexpr const char* field_names[] = {"x", "y", "z", "intensity"};
        int field_offset = 0;
        for (const auto& name : field_names) {
            const auto field = pcl.add_fields();
            field->set_name(name);
            field->set_offset(field_offset);
            field->set_type(foxglove::PackedElementField_NumericType_FLOAT32);
            field_offset += sizeof(float);
        }

        // Reserve enough space for all of our points.
        auto num_points = points.size();
        pcl.mutable_data()->append(num_points * 4 * sizeof(float), '\0');
        pcl.set_frame_id("pointcloud");

        // Position the pointclouds in the center of their coordinate frame.
        auto* pose = pcl.mutable_pose();
        auto* position = pose->mutable_position();
        position->set_x(0);
        position->set_y(0);
        position->set_z(0);
        auto* orientation = pose->mutable_orientation();
        orientation->set_x(0);
        orientation->set_y(0);
        orientation->set_z(0);
        orientation->set_w(1);
        mcap::Timestamp start_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
        auto pcl_timestamp = pcl.mutable_timestamp();
        pcl_timestamp->set_seconds(start_time / 1e9);
        pcl_timestamp->set_nanos(start_time % static_cast<int64_t>(1e9));
        size_t offset = 0;
        for (const auto& point : points) {
            auto* data = pcl.mutable_data()->data();
            memcpy(&data[offset], &point.x, sizeof(float));
            offset += sizeof(float);
            memcpy(&data[offset], &point.y, sizeof(float));
            offset += sizeof(float);
            memcpy(&data[offset], &point.z, sizeof(float));
            offset += sizeof(float);
            memcpy(&data[offset], &point.intensity, sizeof(float));
            offset += sizeof(float);
        }
        std::string serialized = pcl.SerializeAsString();
        mcap::Message msg;
        msg.channelId = point_cloud_channel_id_;
        msg.sequence = frame->frameId();
        msg.publishTime = start_time;
        msg.logTime = start_time;
        msg.data = reinterpret_cast<const std::byte*>(serialized.data());
        msg.dataSize = serialized.size();
        const auto res = point_cloud_writer_->write(msg);
        if (!res.ok()) {
            LOG_ERROR("Failed to write point cloud: {}", res.message);
        }
    }
}

void RecorderImpl::enableRecord(bool enable) { enable_record_ = enable; }

void RecorderImpl::recordThread() {
    // record parameters, only need call once
    while (enable_record_) {
        // take frame from queue
        std::shared_ptr<Frame> frame;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return !frame_queue_.empty(); });
            frame = frame_queue_.front();
            frame_queue_.pop();
        }
        // record frame
        recordFrame(frame);
    }
}

uint32_t RecorderImpl::generateObserverId() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int32_t> dis(
        1, std::numeric_limits<int32_t>::max());
    return dis(gen);
}

}  // namespace ob_lidar::detail
