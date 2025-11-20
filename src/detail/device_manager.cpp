#include "device_manager.hpp"

#include <nlohmann/json.hpp>
#include <sstream>
#include <toml++/toml.hpp>

#include "./minimal_ping.hpp"
#include "device.hpp"

namespace ob_lidar::detail {
std::shared_ptr<Logger> DeviceManagerImpl::logger_ = nullptr;

void DeviceManagerImpl::setLogCallback(const logMessageCallback &callback) {
    log_message_callback_ = callback;
    if (logger_) {
        logger_->setLogCallback(callback);
    }
}

DeviceManagerImpl::DeviceManagerImpl(const std::string &config_file) {
    auto log_config = std::make_shared<LoggerConfig>();
    try {
        // Load the TOML configuration file
        auto config = toml::parse_file(config_file);
        if (auto logging = config["logging"].as_table()) {
            std::stringstream ss;
            ss << toml::json_formatter(*logging);
            auto logging_config_str = ss.str();

            log_config->loadFromJsonString(logging_config_str);
        } else {
            log_config = std::make_shared<LoggerConfig>();
        }
        if (!logger_) {
            logger_ = std::make_shared<Logger>(log_config);
        }
        // Access devices
        if (auto device_config_table = config["device"].as_table()) {
            for (const auto &[_, device_config_node] : *device_config_table) {
                std::stringstream ss;
                ss << toml::json_formatter(device_config_node);
                const auto json_config_str = ss.str();
                try {
                    auto json = nlohmann::json::parse(json_config_str);
                    auto device_name = json["name"].get<std::string>();
                    auto model_name = json["model"].get<std::string>();
                    auto network_config = json["network"];
                    DeviceConfigBuilder builder;
                    auto device_config =
                        builder.setDeviceName(device_name)
                            .setType(model_name)
                            .setNetworkConfigJsonString(network_config.dump(2))
                            .build();
                    auto device = DeviceFactory::create(device_config);
                    addDevice(device);

                } catch (const nlohmann::json::parse_error &err) {
                    LOG_ERROR("Failed to parse device config: {}", err.what());
                }
            }
        }

    } catch (const toml::parse_error &err) {
        LOG_ERROR("Failed to parse config file: {}", err.what());
    }
    LOG_INFO("DriverImpl initialized by config file: {}", config_file);
}

DeviceManagerImpl::DeviceManagerImpl() {
    auto log_config = std::make_shared<LoggerConfig>();
    if (!logger_) {
        logger_ = std::make_shared<Logger>(log_config);
    }
    LOG_INFO("DriverImpl initialized by default constructor");
}

DeviceManagerImpl::~DeviceManagerImpl() {
    stop();
    // stop discovery
    if (mdns_discovery_service_) {
        mdns_discovery_service_->stop();
    }
    check_device_connection_ = false;
    if (check_device_connection_thread_ &&
        check_device_connection_thread_->joinable()) {
        check_device_connection_thread_->join();
    }
}

void DeviceManagerImpl::discovery() {
    mdns_discovery_service_ = std::make_unique<MDNSDiscoveryService>();
    mdns_discovery_service_->start(
        [this](const std::string &ip, const uint16_t &port) {
            device_last_seen_[ip] = std::chrono::high_resolution_clock::now();
            if (on_device_connected_ && !discovered_devices_.count(ip)) {
                discovered_devices_[ip] = port;
                on_device_connected_(ip, port);
            }
        });
}

void DeviceManagerImpl::checkDeviceConnection() {
    // check last seen time of devices
    while (check_device_connection_) {
        for (auto &[ip, last_seen] : device_last_seen_) {
            if (!discovered_devices_.count(ip)) {
                continue;
            }
            auto now = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                now - last_seen);
            // device disconnected
            if (discovered_devices_.count(ip) && duration.count() >= 3) {
                auto port = discovered_devices_[ip];
                LOG_INFO("Device {}:{} disconnected", ip, port);
                if (on_device_disconnected_) {
                    on_device_disconnected_(ip, port);
                }
                std::cout << "erase device " << ip << std::endl;
                LOG_INFO("erase device {} from discovered_devices_", ip);
                discovered_devices_.erase(ip);
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void DeviceManagerImpl::setOnDeviceChangedCallback(
    const onDeviceConnectedCallback &on_device_connected,
    const onDeviceDisconnectedCallback &on_device_disconnected) {
    on_device_connected_ = on_device_connected;
    on_device_disconnected_ = on_device_disconnected;
    // if callback is set, start discovery and check device connection
    if (on_device_connected_ || on_device_disconnected_) {
        check_device_connection_ = true;
        check_device_connection_thread_ = std::make_shared<std::thread>(
            &DeviceManagerImpl::checkDeviceConnection, this);
        discovery();
    }
}

std::vector<std::shared_ptr<Device>> DeviceManagerImpl::getDevices() {
    std::vector<std::shared_ptr<Device>> device_ptrs;
    for (const auto &device : devices_) {
        device_ptrs.push_back(device.second);
    }
    return device_ptrs;
}

std::shared_ptr<Device> DeviceManagerImpl::getDevice(
    const std::string &device_name) {
    const auto &device = devices_.find(device_name);
    if (device != devices_.end()) {
        return device->second;
    }
    return nullptr;
}

void DeviceManagerImpl::addDevice(std::shared_ptr<Device> device) {
    if (devices_.find(device->getName()) != devices_.end()) {
        return;
    }
    LOG_INFO("add device {}", device->getName());
    devices_[device->getName()] = device;
}

void DeviceManagerImpl::removeDevice(const std::string &device_name) {
    if (devices_.find(device_name) == devices_.end()) {
        LOG_ERROR("device {} not found", device_name);
        return;
    }
    devices_.erase(device_name);
}

void DeviceManagerImpl::start() {
    LOG_INFO("start all devices");
    for (auto &[device_name, device] : devices_) {
        const auto status = device->start();
        if (status != Status::OK) {
            LOG_ERROR("failed to start device {}", device_name);
        }
    }
}

void DeviceManagerImpl::stop() {
    LOG_INFO("stop all devices");
    for (auto &[device_name, device] : devices_) {
        const auto status = device->stop();
        if (status != Status::OK) {
            LOG_ERROR("failed to stop device {}", device_name);
        }
    }
    LOG_INFO("stop all devices done");
}

}  // namespace ob_lidar::detail
