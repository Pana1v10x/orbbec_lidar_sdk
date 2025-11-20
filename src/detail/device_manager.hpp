#pragma once

#include <spdlog/spdlog.h>

#include <memory>
#include <uvw.hpp>
#include <vector>

#include "../logger.hpp"
#include "discovery.hpp"
#include "orbbec_lidar/config.hpp"
#include "orbbec_lidar/device.hpp"

namespace ob_lidar::detail {
using time_point_t = std::chrono::high_resolution_clock::time_point;
class DeviceManagerImpl {
   public:
    explicit DeviceManagerImpl(const std::string &config_file);

    void enableDiscovery(bool enable);

    void setLogCallback(const logMessageCallback &callback);

    void setOnDeviceChangedCallback(
        const onDeviceConnectedCallback &on_device_connected,
        const onDeviceDisconnectedCallback &on_device_disconnected);

    DeviceManagerImpl();

    ~DeviceManagerImpl();

    std::vector<std::shared_ptr<Device>> getDevices();

    std::shared_ptr<Device> getDevice(const std::string &device_name);

    void addDevice(std::shared_ptr<Device>);

    void removeDevice(const std::string &device_name);

    void start();

    void stop();

    static std::shared_ptr<Logger> logger_;

    void discovery();

    // check connection status of devices
    void checkDeviceConnection();

   private:
    std::map<std::string, std::shared_ptr<Device>> devices_;
    onDeviceConnectedCallback on_device_connected_;
    onDeviceDisconnectedCallback on_device_disconnected_;
    std::shared_ptr<std::thread> check_device_connection_thread_ = nullptr;
    std::atomic_bool check_device_connection_{false};
    std::unique_ptr<MDNSDiscoveryService> mdns_discovery_service_ = nullptr;
    // ip -> last seen time
    std::map<std::string, time_point_t> device_last_seen_;
    std::map<std::string, uint16_t> discovered_devices_;
    logMessageCallback log_message_callback_;
};

}  // namespace ob_lidar::detail
