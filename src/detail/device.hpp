#pragma once

#include <spdlog/fmt/bundled/format.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <orbbec_lidar/config.hpp>
#include <orbbec_lidar/frame.hpp>
#include <orbbec_lidar/option.hpp>
#include <orbbec_lidar/types.hpp>
#include "orbbec_lidar/option_utils.hpp"

#include <queue>

#include "capability.hpp"
#include "frame.hpp"
#include "lidar_packet_parser.hpp"
#include "multi_line_capability.hpp"
#include "multi_line_parameter.hpp"
#include "network_comm.hpp"
#include "option_mapper.hpp"
#include "request.hpp"
#include "single_line_capability.hpp"
#include "single_line_parameter.hpp"
#include "spdlog/formatter.h"

namespace ob_lidar::detail {

// one scan data size
static constexpr int kMaxScanSize = 3600 * 4 + 5;

class DeviceImpl {
   public:
    explicit DeviceImpl(const std::string &config_file);

    explicit DeviceImpl(const std::shared_ptr<DeviceConfig> &config);

    ~DeviceImpl() noexcept;

    void init();

    void loadDeviceInfo();

    Status initiateConnection();

    void registerDataCallbacks();

    DeviceInfo getInfo();

    Status start();

    Status stop();

    Status start(const std::shared_ptr<StreamConfig> &config,
                 const FrameCallback &callback);

    void registerFrameObserver(uint32_t observer_id, const FrameCallback &callback);

    void unregisterFrameObserver(uint32_t observer_id);

    [[nodiscard]] bool isOptionSupported(const LidarOption &option) const;

    [[nodiscard]] bool isOptionSupported(
        const LidarOption &type, const LidarOptionPermission &permission) const;

    [[nodiscard]] bool isStreamSupported(const LidarStreamType &type) const;

    [[nodiscard]] LidarType getType() const;

    [[nodiscard]] std::string getName() const;

    void setDeviceName(const std::string &name);

    Status setOption(const LidarOption &option, const void *value,
                     size_t value_size);

    Status setOption(const uint16_t &address, const void *value,
                     size_t value_size);

    template <typename T>
    Status setOption(const LidarOption &option, const T &value) {
        if (!isOptionSupported(option)) {
            return Status::ERR;
        }
        if constexpr (std::is_pod_v<T>) {
            return setOption(option, &value, sizeof(value));
        } else if constexpr (std::is_same_v<T, std::string>) {
            return setOption(option, value.c_str(), value.size());
        } else {
            return Status::ERR;
        }
    }

    Status getOption(const LidarOption &option, void *value, size_t value_size,
                     size_t *size_read);

    Status getOption(const uint16_t &address, void *value, size_t value_size,
                     size_t *size_read);

    template <typename T>
    T getOption(const LidarOption &option) {
        // Check if the option is supported
        LOG_INFO("Getting option: {}", asStringLiteral(option));
        if (!isOptionSupported(option)) {
            const auto err_msg = spdlog::fmt_lib::format(
                "Option {} is not supported", asStringLiteral(option));
            throw std::runtime_error(err_msg);
        }
        if constexpr (std::is_pod_v<T>) {
            T value;
            size_t size_read = 0;
            auto status = getOption(option, &value, sizeof(value), &size_read);
            if (status != Status::OK) {
                const auto err_msg = spdlog::fmt_lib::format(
                    "Failed to get option: {} with error: {}",
                    asStringLiteral(option),asStringLiteral(status));
                throw std::runtime_error(err_msg);
            }
            return value;
        } else {
            const auto err_msg = spdlog::fmt_lib::format(
                "Option {} is not supported", asStringLiteral(option));
            throw std::runtime_error(err_msg);
        }
    }

    bool isNetworkReachable() const;

    std::shared_ptr<Frame> waitForFrame(
        const std::chrono::milliseconds &timeout);

   private:
    void setCommunicationProtocol(const LidarProtocolType &protocol);

    void configureScanBlockSizeBasedOnWorkMode();

    void sendCommandAndWaitForResponse(uint16_t command_id, const void *req,
                                       size_t req_size, void *resp,
                                       size_t *resp_size);

    void genericDataCallback(const uint8_t *data, size_t size);

    void onPointCloudDataReceived(const uint8_t *data, size_t size);

    void onScanDataReceived(const uint8_t *data, size_t size);

    void publishScanFrame(int frame_index,
                          const std::chrono::nanoseconds &timestamp,
                          const LaserScanMeta &meta_data,
                          const std::vector<uint16_t> &ranges,
                          const std::vector<uint16_t> &intensities);

    static void extractScanData(const std::vector<uint8_t> &scan_data,
                                std::vector<uint16_t> &ranges,
                                std::vector<uint16_t> &intensities);

    void mergeScanData(std::vector<uint16_t> &ranges,
                       std::vector<uint16_t> &intensities);

    void clearScanData();

    void onCommandResponseReceived(const uint8_t *data, size_t size);

    void heartbeatThread();

   private:
    // device name unique
    std::string device_name_;
    // lidar type, single line or multi line
    LidarType type_ = LidarType::UNKNOWN;
    // device config
    std::shared_ptr<DeviceConfig> config_ = nullptr;
    // network config
    std::shared_ptr<NetworkConfig> network_config_ = nullptr;
    std::atomic_bool is_started_{false};
    // enable heartbeat, default is true.
    bool enable_heartbeat_{true};
    std::shared_ptr<std::thread> heartbeat_thread_ = nullptr;
    int max_heartbeat_fail_count_{3};
    int heartbeat_fail_count_{0};
    long heartbeat_interval_ms_{1000};
    std::atomic_bool is_network_reachable_{false};
    // device info
    DeviceInfo device_info_;
    // frame callback
    FrameCallback frame_callback_;
    // network communication
    std::unique_ptr<NetworkComm> network_comm_ = nullptr;
    std::mutex network_comm_mutex_;
    // lidar capabilities
    std::shared_ptr<LidarCapabilitiesInterface> capability_ = nullptr;
    // frame observers
    std::map<uint32_t, FrameCallback> frame_observers_;

    char command_resp_packet_buffer_[128]{};
    size_t command_resp_packet_size_ = 0;
    std::atomic_bool command_response_received_{false};
    // command packet mutex
    std::mutex command_packet_mutex_;
    // command packet condition variable
    std::condition_variable command_packet_cv_;
    // current scan speed, if you use hz, you can convert it to RPM, 1hz = 60RPM
    int scan_speed_ = 0;
    // max scan block size
    size_t max_scan_block_size_ = 0;
    // block point size
    size_t block_point_size_ = 0;
    // current block index
    int current_block_index_ = 0;
    uint8_t scan_data_buffer_[kMaxScanSize]{};
    size_t scan_data_size_ = 0;  // sum of scan blocks size
    // wait all scan blocks
    bool wait_all_scan_blocks_ = true;
    // min angle
    double min_angle_ = 360;
    // max angle
    double max_angle_ = 0;
    // for async get frame
    std::queue<std::shared_ptr<Frame>> frame_queue_;
    std::mutex frame_queue_mutex_;
    std::condition_variable frame_queue_cv_;
    int max_frame_queue_size_ = 10;
    std::atomic_bool is_initialized_{false};
};

}  // namespace ob_lidar::detail
