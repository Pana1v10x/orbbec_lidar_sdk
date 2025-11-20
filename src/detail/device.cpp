#include "device.hpp"

#include <chrono>
#include <memory>
#include <set>
#include <utils.hpp>

#include "../logger.hpp"
#include "check.hpp"
#include "frame.hpp"
#include "orbbec_lidar/frame.hpp"
#include "orbbec_lidar/option_utils.hpp"
#include "spdlog/sinks/null_sink.h"

namespace ob_lidar::detail {
static constexpr int LASER_SCAN_BLOCK_LENGTH_10HZ = 378;
static constexpr int LASER_SCAN_BLOCK_LENGTH_15HZ = 200;
static constexpr int LASER_SCAN_BLOCK_LENGTH_20HZ_CALIB = 240;
static constexpr int LASER_SCAN_BLOCK_LENGTH_20HZ = 150;
static constexpr int LASER_SCAN_BLOCK_LENGTH_25HZ = 120;
static constexpr int LASER_SCAN_BLOCK_LENGTH_30HZ = 100;

static constexpr int LASER_SCAN_BLOCK_TYPE_10HZ = 0x00;
static constexpr int LASER_SCAN_BLOCK_TYPE_15HZ = 0x01;
static constexpr int LASER_SCAN_BLOCK_TYPE_20HZ = 0x02;
static constexpr int LASER_SCAN_BLOCK_TYPE_25HZ = 0x03;
static constexpr int LASER_SCAN_BLOCK_TYPE_30HZ = 0x04;

DeviceImpl::DeviceImpl(const std::string& config_file)
    : config_(std::make_shared<DeviceConfig>(config_file)) {
    init();
}

DeviceImpl::DeviceImpl(const std::shared_ptr<DeviceConfig>& config)
    : config_(config) {
    init();
}

void DeviceImpl::loadDeviceInfo() {
    // get serial number
    char serial_number[64];
    constexpr auto serial_number_option = OB_LIDAR_OPTION_SERIAL_NUMBER;
    size_t size_read = 0;
    getOption(serial_number_option, serial_number, sizeof(serial_number),
              &size_read);

    serial_number[size_read] = '\0';
    device_info_.serial_number = serial_number;
    LOG_INFO("Serial number: {}", device_info_.serial_number);

    // get firmware version
    char firmware_version[64];
    constexpr auto firmware_version_option =
        LidarOption::OB_LIDAR_OPTION_FIRMWARE_VERSION;
    size_read = 0;
    getOption(firmware_version_option, firmware_version,
              sizeof(firmware_version), &size_read);
    device_info_.firmware_version = firmware_version;
    LOG_INFO("Firmware version: {}", device_info_.firmware_version);
    // get fpga version
    char fpga_version[64];
    constexpr auto fpga_version_option = OB_LIDAR_OPTION_FPGA_VERSION;
    size_read = 0;
    getOption(fpga_version_option, fpga_version, sizeof(fpga_version),
              &size_read);
    fpga_version[size_read] = '\0';
    device_info_.fpga_version = fpga_version;
    LOG_INFO("FPGA version: {}", device_info_.fpga_version);
    // get product model
    char product_model[64];
    constexpr auto product_model_option = OB_LIDAR_OPTION_PRODUCT_MODEL;
    size_read = 0;
    getOption(product_model_option, product_model, sizeof(product_model),
              &size_read);
    product_model[size_read] = '\0';
    device_info_.model = product_model;
    LOG_INFO("Product model: {}", device_info_.model);
}

void DeviceImpl::registerDataCallbacks() {
    CHECK_NOTNULL(network_config_);
    if (network_config_->isSinglePort()) {
        LOG_INFO("Registering data callback for single port");
        network_comm_->setOnDataCallback(
            [this](const uint8_t* data, size_t size) {
                genericDataCallback(data, size);
            });
    } else {
        // command channel
        LOG_INFO("Registering data callback for command channel");
        network_comm_->setOnDataCallback(
            LidarChannelType::COMMAND,
            [this](const uint8_t* data, size_t size) {
                genericDataCallback(data, size);
            });
        // TODO: point cloud channel
        // TODO: sphere point cloud channel
        // TODO: imu channel
        // TODO: log channel
    }
}

Status DeviceImpl::initiateConnection() {
    LOG_INFO("Connecting to device: {}", device_name_);
    constexpr auto connection_option =
        OB_LIDAR_OPTION_INITIATE_DEVICE_CONNECTION;
    const uint32_t command_code = htonl(0x12345678);
    return setOption(connection_option, &command_code, sizeof(command_code));
}

DeviceImpl::~DeviceImpl() {
    LOG_INFO("Destroying device: {}", device_name_);
    if (is_started_) {
        stop();
    }
    is_started_ = false;
    is_initialized_ = false;
    if (heartbeat_thread_ != nullptr && heartbeat_thread_->joinable()) {
        enable_heartbeat_ = false;
        heartbeat_thread_->join();
    }
    LOG_INFO("Device {} destroyed", device_name_);
}

void DeviceImpl::init() {
    CHECK_NOTNULL(config_);
    device_name_ = config_->getDeviceName();
    LOG_INFO("Initializing device: {}", device_name_);
    type_ = config_->getLidarType();
    device_info_.device_name = device_name_;
    device_info_.lidar_type = asStringLiteral(type_);
    network_config_ = config_->getNetworkConfig();
    device_info_.ip = network_config_->getIp();
    device_info_.port = network_config_->getPort();
    CHECK_NOTNULL(network_config_);
    network_comm_ = NetworkCommFactory::create(config_->getNetworkConfig());
    CHECK_NOTNULL(network_comm_);

    if (type_ == LidarType::SINGLE_LINE) {
        capability_ = std::make_shared<SingleLineCapabilities>();
    } else if (type_ == LidarType::MULTI_LINE) {
        capability_ = std::make_shared<MultiLineCapabilities>();
    } else {
        throw std::runtime_error("Unsupported lidar model");
    }
    registerDataCallbacks();
    auto status = initiateConnection();
    if (status == Status::OK) {
        LOG_INFO("Connected to device: {}", device_name_);
    } else {
        LOG_ERROR("Failed to connect to device: {}", device_name_);
    }
    loadDeviceInfo();
    if (type_ == LidarType::SINGLE_LINE) {
        configureScanBlockSizeBasedOnWorkMode();
    }
    is_initialized_ = true;
    const auto& device_params = config_->getParams();
    LOG_INFO("enable_heartbeat: {}", device_params.enable_heartbeat);
    if (device_params.enable_heartbeat) {
        max_heartbeat_fail_count_ = device_params.max_heartbeat_fail_allowed;
        heartbeat_interval_ms_ = device_params.heartbeat_interval_ms;
        heartbeat_thread_ =
            std::make_shared<std::thread>(&DeviceImpl::heartbeatThread, this);
    }
    LOG_INFO("Device {} initialized", device_name_);
}

DeviceInfo DeviceImpl::getInfo() { return device_info_; }

Status DeviceImpl::start() {
    LOG_INFO("Starting device: {}", device_name_);
    constexpr auto streaming_option = OB_LIDAR_OPTION_ENABLE_STREAMING;
    // stop streaming first
    const int disable_streaming = htonl(0);
    setOption(streaming_option, &disable_streaming, sizeof(disable_streaming));
    const int enable_streaming = htonl(1);
    // TODO: set stream config, such as frame rate, scan direction, etc.
    const auto status = setOption(streaming_option, &enable_streaming,
                                  sizeof(enable_streaming));
    if (status == Status::OK) {
        is_started_ = true;
        LOG_INFO("Device {} started", device_name_);
    } else {
        LOG_ERROR("Failed to start device: {}", device_name_);
    }
    return status;
}

Status DeviceImpl::stop() {
    if (!is_started_) {
        LOG_INFO("Device {} is not started", device_name_);
        return Status::OK;
    }
    constexpr auto streaming_option = OB_LIDAR_OPTION_ENABLE_STREAMING;
    const int disable_streaming = htonl(0);
    const auto status = setOption(streaming_option, &disable_streaming,
                                  sizeof(disable_streaming));
    if (status == Status::OK) {
        is_started_ = false;
        LOG_INFO("Device {} stopped", device_name_);
    } else {
        LOG_ERROR("Failed to stop device: {}", device_name_);
    }
    return status;
}

Status DeviceImpl::start(const std::shared_ptr<StreamConfig>& config,
                         const FrameCallback& callback) {
    frame_callback_ = callback;
    // FIXME: need handle IMU and log stream in the future
    auto stream_list = {LidarStreamType::SCAN,
                        LidarStreamType::SPHERE_POINT_CLOUD,
                        LidarStreamType::POINT_CLOUD};
    // get scan speed
    int scan_speed = 0;
    size_t size_read = 0;
    getOption(OB_LIDAR_OPTION_SCAN_SPEED, &scan_speed, sizeof(scan_speed),
              &size_read);
    scan_speed = ntohl(scan_speed);
    LOG_INFO("Scan speed: {} RPM", scan_speed);
    for (const auto& stream : stream_list) {
        if (config->isStreamEnabled(stream)) {
            double freq = config->getStreamFrequency(stream);
            if (capability_->isStreamFreqSupported(stream, freq)) {
                int value = htonl(static_cast<int>(freq * 60));
                if (freq * 60 == scan_speed) {
                    break;
                }
                auto status = setOption(OB_LIDAR_OPTION_SCAN_SPEED, &value,
                                        sizeof(value));
                if (status != Status::OK) {
                    LOG_ERROR("Failed to set scan speed: {}", freq);
                    return status;
                }
                // if find the first supported stream, break
                break;
            }
        }
    }
    return start();
}

void DeviceImpl::registerFrameObserver(uint32_t observer_id,
                                       const FrameCallback& callback) {
    frame_observers_[observer_id] = callback;
}

void DeviceImpl::unregisterFrameObserver(uint32_t observer_id) {
    if (frame_observers_.find(observer_id) != frame_observers_.end()) {
        frame_observers_.erase(observer_id);
    }
}

bool DeviceImpl::isOptionSupported(const LidarOption& option) const {
    return capability_->isOptionSupported(option);
}

bool DeviceImpl::isOptionSupported(
    const LidarOption& type, const LidarOptionPermission& permission) const {
    return capability_->isOptionSupported(type, permission);
}

bool DeviceImpl::isStreamSupported(const LidarStreamType& type) const {
    return capability_->isStreamSupported(type);
}

LidarType DeviceImpl::getType() const { return type_; }

std::string DeviceImpl::getName() const { return device_name_; }

void DeviceImpl::setDeviceName(const std::string& name) {
    device_name_ = name;
    device_info_.device_name = name;
}

void DeviceImpl::sendCommandAndWaitForResponse(uint16_t command_id,
                                               const void* req, size_t req_size,
                                               void* resp, size_t* resp_size) {
    // Send data
    {
        command_response_received_ = false;
        std::unique_lock lock(network_comm_mutex_);
        CHECK_NOTNULL(network_comm_);
        CHECK_NOTNULL(network_config_);
        Request request(command_id);
        auto payload = request.serialize(req, req_size);
        if (network_config_->isSinglePort()) {
            network_comm_->sendData(payload);
        } else {
            network_comm_->sendData(LidarChannelType::COMMAND, payload);
        }
    }
    // Wait for response
    {
        std::unique_lock lock(command_packet_mutex_);
        const std::chrono::seconds timeout(3);
        if (!command_packet_cv_.wait_for(lock, timeout, [this]() {
                return command_response_received_.load();
            })) {
            auto command = static_cast<LidarCommandInterface>(command_id);
            LOG_ERROR("Timeout waiting for response {}",
                      asStringLiteral(command));
            throw std::runtime_error("Timeout waiting for response");
        }
        if (resp != nullptr && resp_size != nullptr) {
            *resp_size = command_resp_packet_size_;
            memcpy(resp, command_resp_packet_buffer_, req_size);
        }
    }
}

Status DeviceImpl::setOption(const LidarOption& option, const void* value,
                             size_t value_size) {
    LOG_INFO("Setting option: {}", asStringLiteral(option));
    auto command_interface = LidarOptionMapper::getSetCommandInterface(option);
    if (command_interface == LidarCommandInterface::UNKNOWN) {
        LOG_ERROR("Unknown option: {}", static_cast<int>(option));
        return Status::ERR;
    }
    auto command_id = static_cast<uint16_t>(command_interface);
    try {
        sendCommandAndWaitForResponse(command_id, value, value_size, nullptr,
                                      nullptr);
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to set option: {} with error: {}",
                  asStringLiteral(option), e.what());
        return Status::ERR;
    }
    // if is set work mode option, set max block size.
    if (option == OB_LIDAR_OPTION_WORK_MODE &&
        type_ == LidarType::SINGLE_LINE) {
        auto work_mode = *static_cast<const uint32_t*>(value);
        if (work_mode == 3) {
            // I don't know why, but this is the max scan block size for 20hz
            max_scan_block_size_ = 90;
            block_point_size_ = LASER_SCAN_BLOCK_LENGTH_20HZ_CALIB;
        } else {
            // 10hz
            max_scan_block_size_ = 18;
            block_point_size_ = LASER_SCAN_BLOCK_LENGTH_20HZ;
        }
    }
    return Status::OK;
}

Status DeviceImpl::setOption(const uint16_t& address, const void* value,
                             size_t value_size) {
    try {
        sendCommandAndWaitForResponse(address, value, value_size, nullptr,
                                      nullptr);
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to set option: {}", e.what());
        return Status::ERR;
    }
    return Status::OK;
}

Status DeviceImpl::getOption(const LidarOption& option, void* value,
                             size_t value_size, size_t* size_read) {
    LOG_DEBUG("Getting option: {}", asStringLiteral(option));
    const auto command_interface =
        LidarOptionMapper::getGetCommandInterface(option);
    if (command_interface == LidarCommandInterface::UNKNOWN) {
        LOG_ERROR("Unknown option: {}", static_cast<int>(option));
        return Status::ERR;
    }
    const auto command_id = static_cast<uint16_t>(command_interface);
    try {
        sendCommandAndWaitForResponse(command_id, value, value_size, value,
                                      size_read);
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to get option: {} with error: {}",
                  asStringLiteral(option), e.what());
        return Status::ERR;
    }
    return Status::OK;
}

Status DeviceImpl::getOption(const uint16_t& address, void* value,
                             size_t value_size, size_t* size_read) {
    try {
        sendCommandAndWaitForResponse(address, value, value_size, value,
                                      size_read);
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to get option: {}", e.what());
        return Status::ERR;
    }
    return Status::OK;
}

#define COMMAND_TYPE_RESPONSE_HEADER 0xFE01
#define DATA_TYPE_RESPONSE_HEADER 0x534D

void DeviceImpl::genericDataCallback(const uint8_t* data, size_t size) {
    uint16_t header;
    if (size < 2) {
        LOG_ERROR("Data size is too small");
        return;
    }
    memcpy(&header, data, sizeof(uint16_t));
    if (header == COMMAND_TYPE_RESPONSE_HEADER) {
        onCommandResponseReceived(data, size);
    } else if (header == DATA_TYPE_RESPONSE_HEADER &&
               type_ == LidarType::MULTI_LINE) {
        onPointCloudDataReceived(data, size);
    } else if (header == DATA_TYPE_RESPONSE_HEADER &&
               type_ == LidarType::SINGLE_LINE) {
        try {
            onScanDataReceived(data, size);
        } catch (const std::exception& e) {
            LOG_ERROR("Failed to process scan data: {}", e.what());
        }
    }
}

void DeviceImpl::onPointCloudDataReceived(const uint8_t* data, size_t size) {
    (void)this;
    (void)data;
    (void)size;
}

void DeviceImpl::onScanDataReceived(const uint8_t* data, size_t size) {
    if (!is_started_) {
        return;
    }
    if (type_ != LidarType::SINGLE_LINE) {
        LOG_ERROR("Only model single-line lidar is supported scan data");
        return;
    }
    // TODO: too much redundant code, need refactor
    // Parse the incoming data
    auto scan_data_packet =
        LidarPacketParser::parse<LidarPacketType::SINGLE_LINE>(data, size);
    auto header = scan_data_packet->header();
    auto frame_index = header->getFrameIndex();
    const auto timestamp = std::chrono::nanoseconds(header->getTimestamp());
    auto payload = scan_data_packet->payload();
    auto payload_size = scan_data_packet->payloadSize();
    uint8_t block_index = header->getBlockIndex();

    // Use angle scaling to calculate actual angles
    if (!wait_all_scan_blocks_) {
        min_angle_ = header->getStartAngle();
        max_angle_ = header->getEndAngle();
        LaserScanMeta meta_data{};
        meta_data.sync_mode = header->getSyncMode();
        meta_data.start_angle = min_angle_;
        meta_data.end_angle = max_angle_;
        meta_data.angle_resolution = header->getAngleResolution();
        meta_data.contaminated_angle = header->getContaminatedAngle();
        meta_data.contaminated_level = header->getContaminatedLevel();
        std::vector<uint16_t> ranges, intensities;

        size_t points_size = payload_size / 4;
        ranges.resize(points_size);
        intensities.resize(points_size);

        for (size_t i = 0; i < payload_size; i += 4) {
            uint16_t range =
                ntohs(*reinterpret_cast<const uint16_t*>(payload_size + i));
            uint16_t intensity =
                ntohs(*reinterpret_cast<const uint16_t*>(payload_size + i + 2));
            ranges[i / 4] = range * 2;  // Convert to millimeters
            intensities[i / 4] = intensity;
        }

        publishScanFrame(frame_index, timestamp, meta_data, ranges,
                         intensities);
        return;
    }
    min_angle_ = std::min<double>(min_angle_, header->getStartAngle());
    max_angle_ = std::max<double>(max_angle_, header->getEndAngle());
    if (block_index == 1) {
        clearScanData();
    }

    // Push the scan data block into the queue
    if (scan_data_size_ + payload_size < sizeof(scan_data_buffer_)) {
        memcpy(scan_data_buffer_ + scan_data_size_, payload, payload_size);
    } else {
        LOG_ERROR(
            "scan_data_size {} + payload_size {} > sizeof(scan_data_buffer_)",
            scan_data_size_, payload_size);
        LOG_ERROR("block_index: {}", block_index);
    }
    auto prev_scan_data_size = scan_data_size_;
    scan_data_size_ += payload_size;
    if (scan_data_size_ > sizeof(scan_data_buffer_)) {
        // print payload size and buffer size
        LOG_ERROR("Payload size: {}", payload_size);
        LOG_ERROR("Buffer size: {}", scan_data_size_);
        // print prev
        LOG_ERROR("Prev scan data size: {}", prev_scan_data_size);
        LOG_ERROR("Scan data buffer overflow");
        LOG_ERROR("Clearing scan data buffer");
        clearScanData();
        return;
    }

    // Ensure that the maximum scan block size is valid
    CHECK(max_scan_block_size_ > 0);

    // If we reached the last block, process and merge the data
    if (block_index == max_scan_block_size_) {
        LaserScanMeta meta_data{};
        meta_data.sync_mode = header->getSyncMode();
        meta_data.start_angle = min_angle_;
        meta_data.end_angle = max_angle_;
        meta_data.angle_resolution = header->getAngleResolution();
        meta_data.contaminated_angle = header->getContaminatedAngle();
        meta_data.contaminated_level = header->getContaminatedLevel();
        std::vector<uint16_t> ranges, intensities;
        mergeScanData(ranges, intensities);
        meta_data.scan_size = ranges.size();
        publishScanFrame(frame_index, timestamp, meta_data, ranges,
                         intensities);
        clearScanData();
    }
}

void DeviceImpl::publishScanFrame(int frame_index,
                                  const std::chrono::nanoseconds& timestamp,
                                  const LaserScanMeta& meta_data,
                                  const std::vector<uint16_t>& ranges,
                                  const std::vector<uint16_t>& intensities) {
    auto frame_impl = std::make_unique<ScanFrameImpl>(ranges.size() * 4);
    frame_impl->setTimestamp(timestamp);
    frame_impl->setFrameType(LidarFrameType::SCAN);
    frame_impl->setFrameId(frame_index);
    frame_impl->setRanges(ranges);
    frame_impl->setIntensities(intensities);
    frame_impl->setMeta(meta_data);

    std::shared_ptr<Frame> frame =
        std::make_shared<ScanFrame>(std::move(frame_impl));
    // push to queue
    std::unique_lock lock(frame_queue_mutex_);
    // if size is greater than max queue size, drop the oldest frame
    if (frame_queue_.size() >= max_frame_queue_size_) {
        frame_queue_.pop();
    }
    frame_queue_.push(frame);
    frame_queue_cv_.notify_all();
    // invoke callback
    if (frame_callback_) {
        frame_callback_(frame);
    }
    for (const auto& [_, observer] : frame_observers_) {
        observer(frame);
    }
}

void DeviceImpl::extractScanData(const std::vector<uint8_t>& scan_data,
                                 std::vector<uint16_t>& ranges,
                                 std::vector<uint16_t>& intensities) {
    size_t points_size = scan_data.size() / 4;
    ranges.resize(points_size);
    intensities.resize(points_size);

    for (size_t i = 0; i < scan_data.size(); i += 4) {
        uint16_t range =
            ntohs(*reinterpret_cast<const uint16_t*>(scan_data.data() + i));
        uint16_t intensity =
            ntohs(*reinterpret_cast<const uint16_t*>(scan_data.data() + i + 2));
        ranges[i / 4] = range * 2;  // Convert to millimeters
        intensities[i / 4] = intensity;
    }
}

void DeviceImpl::mergeScanData(std::vector<uint16_t>& ranges,
                               std::vector<uint16_t>& intensities) {
    size_t points_size = scan_data_size_ / 4;
    ranges.resize(points_size);
    intensities.resize(points_size);
    size_t index = 0;

    for (size_t i = 0; i < scan_data_size_; i += 4) {
        uint16_t range =
            ntohs(*reinterpret_cast<const uint16_t*>(scan_data_buffer_ + i));
        uint16_t intensity = ntohs(
            *reinterpret_cast<const uint16_t*>(scan_data_buffer_ + i + 2));
        ranges[index] = range * 2;  // Convert to millimeters
        intensities[index] = intensity;
        index++;
    }
    scan_data_size_ = 0;
}

void DeviceImpl::clearScanData() {
    scan_data_size_ = 0;
    memset(scan_data_buffer_, 0, sizeof(scan_data_buffer_));
}

void DeviceImpl::onCommandResponseReceived(const uint8_t* data, size_t size) {
    auto command_packet =
        LidarPacketParser::parse<LidarPacketType::COMMAND>(data, size);
    if (!command_packet) {
        LOG_ERROR("Failed to parse command response");
        return;
    }
    auto data_size = command_packet->payloadSize();
    auto data_ptr = command_packet->payload();
    if (data_size > sizeof(command_resp_packet_buffer_)) {
        LOG_ERROR("Command response data size too large: {}", data_size);
        return;
    }
    std::lock_guard lock(command_packet_mutex_);
    memcpy(command_resp_packet_buffer_, data_ptr, data_size);
    command_resp_packet_size_ = data_size;
    command_response_received_ = true;
    command_packet_cv_.notify_all();
}

bool DeviceImpl::isNetworkReachable() const { return is_network_reachable_; }

std::shared_ptr<Frame> DeviceImpl::waitForFrame(
    const std::chrono::milliseconds& timeout) {
    std::unique_lock lock(frame_queue_mutex_);
    // Wait for the frame
    frame_queue_cv_.wait_for(lock, timeout,
                             [this]() { return !frame_queue_.empty(); });
    // if still no frame, return nullptr
    if (frame_queue_.empty()) {
        return std::make_shared<NullFrame>();
    }
    auto frame = frame_queue_.front();
    frame_queue_.pop();
    return frame;
}

void DeviceImpl::setCommunicationProtocol(const LidarProtocolType& protocol) {
    // set data publish protocol, 0: UDP, 1: TCP
    if (protocol != LidarProtocolType::UNKNOWN) {
        constexpr auto protocol_option =
            LidarOption::OB_LIDAR_OPTION_TRANSFER_PROTOCOL;
        auto protocol_value = static_cast<const int>(protocol);
        setOption(protocol_option, &protocol_value, sizeof(protocol_value));
    }
}

void DeviceImpl::configureScanBlockSizeBasedOnWorkMode() {
    // get work mode
    uint32_t work_mode = 0;
    size_t size_read = 0;
    getOption(LidarOption::OB_LIDAR_OPTION_WORK_MODE, &work_mode,
              sizeof(work_mode), &size_read);
    // TODO: refactor this magic stuff
    if (work_mode == 3) {
        // Magic things, I don't know why, but this is the max scan block size
        // for 20hz
        LOG_INFO("Work mode: 3, max scan block size: 90");
        max_scan_block_size_ = 90;
        block_point_size_ = LASER_SCAN_BLOCK_LENGTH_20HZ_CALIB;
    } else {
        LOG_INFO("Work mode: {}, max scan block size: 18", work_mode);
        max_scan_block_size_ = 18;
        block_point_size_ = LASER_SCAN_BLOCK_LENGTH_20HZ;
    }
}

void DeviceImpl::heartbeatThread() {
    LOG_INFO("Starting heartbeat thread");
    while (is_initialized_) {
        // check whether the network is reachable by getting serial number
        char serial_number[64]{};  // NOLINT
        constexpr auto serial_number_option = OB_LIDAR_OPTION_SERIAL_NUMBER;
        size_t size_read = 0;
        const auto status = getOption(serial_number_option, serial_number,
                                      sizeof(serial_number), &size_read);
        if (status != Status::OK) {
            heartbeat_fail_count_++;
            LOG_WARN("send heartbeat failed, fail count: {}",
                     heartbeat_fail_count_);
        } else {
            heartbeat_fail_count_ = 0;
        }
        if (heartbeat_fail_count_ >= max_heartbeat_fail_count_) {
            is_network_reachable_ = false;
            LOG_ERROR("Device is not reachable");
        } else {
            is_network_reachable_ = true;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(heartbeat_interval_ms_));
    }
}

}  // namespace ob_lidar::detail
