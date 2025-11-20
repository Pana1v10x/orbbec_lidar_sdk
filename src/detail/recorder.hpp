#pragma once

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>

#include <atomic>
#include <condition_variable>
#include <mcap/writer.hpp>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "orbbec_lidar/device.hpp"
#include "orbbec_lidar/types.hpp"

namespace foxglove {

/// Builds a FileDescriptorSet of this descriptor and all transitive
/// dependencies, for use as a channel schema.
google::protobuf::FileDescriptorSet BuildFileDescriptorSet(
    const google::protobuf::Descriptor* toplevelDescriptor);

}  // namespace foxglove


namespace ob_lidar::detail {



class RecorderImpl {
   public:
    RecorderImpl(std::shared_ptr<Device> device, const std::string& output_dir);

    ~RecorderImpl();

    void start();

    void stop();

    static uint32_t generateObserverId();

    void recordFrame(std::shared_ptr<Frame> frame) const;

    void frameCallback(std::shared_ptr<Frame> frame);

    void enableRecord(bool enable);

    void recordThread();

   private:
    std::shared_ptr<Device> device_;
    std::string output_dir_;
    std::queue<std::shared_ptr<Frame>> frame_queue_;
    int max_queue_size_ = 64;
    std::shared_ptr<std::thread> record_thread_ = nullptr;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> running_{false};
    std::atomic<bool> enable_record_{false};
    // mcap writer
    std::shared_ptr<mcap::McapWriter> point_cloud_writer_;
    mcap::ChannelId point_cloud_channel_id_;
    // observer id
    uint32_t observer_id_;
};
}  // namespace ob_lidar::detail
