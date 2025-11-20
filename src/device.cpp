#include "detail/device.hpp"

#include <orbbec_lidar/device.hpp>
#include <uvw.hpp>
#ifdef _WIN32
#include <winsock2.h>
#elif __linux__
#include <arpa/inet.h>
#endif
namespace ob_lidar {

Device::Device(std::unique_ptr<detail::DeviceImpl> impl)
    : impl_(std::move(impl)) {}

Device::~Device() noexcept = default;

DeviceInfo Device::getInfo() { return impl_->getInfo(); }

Status Device::start() { return impl_->start(); }

Status Device::stop() { return impl_->stop(); }

Status Device::start(const std::shared_ptr<StreamConfig> &config,
                     const FrameCallback &callback) {
    return impl_->start(config, callback);
}

void Device::registerFrameObserver(uint32_t observer_id,
                                   const FrameCallback &callback) {
    impl_->registerFrameObserver(observer_id, callback);
}

void Device::unregisterFrameObserver(uint32_t observer_id) {
    impl_->unregisterFrameObserver(observer_id);
}

Status Device::setUIntOption(const LidarOption &option, uint32_t value) {
    // convert to network order
    return impl_->setOption<uint32_t>(option, htonl(value));
}

Status Device::setUint16Option(const LidarOption &option, uint16_t value) {
    // convert to network order
    return impl_->setOption<uint16_t>(option, htons(value));
}

Status Device::setBoolOption(const LidarOption &option, bool value) {
    return impl_->setOption<bool>(option, value);
}

Status Device::setFloatOption(const LidarOption &option, float value) {
    return impl_->setOption<float>(option, value);
}

Status Device::setOption(const LidarOption &option, const void *value,
                         size_t value_size) {
    return impl_->setOption(option, value, value_size);
}

Status Device::setOption(const uint16_t &address, const void *value,
                         size_t value_size) {
    return impl_->setOption(address, value, value_size);
}

int Device::getIntOption(const LidarOption &option) {
    // convert to host order
    return ntohl(impl_->getOption<int>(option));
}

uint16_t Device::getUint16Option(const LidarOption &option) {
    // convert to host order
    return ntohs(impl_->getOption<uint16_t>(option));
}

bool Device::getBoolOption(const LidarOption &option) {
    return impl_->getOption<bool>(option);
}

float Device::getFloatOption(const LidarOption &option) {
    return impl_->getOption<float>(option);
}

Status Device::getOption(const LidarOption &option, void *value,
                         size_t value_size, size_t *size_read) {
    return impl_->getOption(option, value, value_size, size_read);
}

Status Device::getOption(const uint16_t &address, void *value, size_t size,
                         size_t *size_read) {
    return impl_->getOption(address, value, size, size_read);
}

bool Device::isOptionSupported(const LidarOption &option) {
    return impl_->isOptionSupported(option);
}

bool Device::isOptionSupported(const LidarOption &type,
                               const LidarOptionPermission &permission) {
    return impl_->isOptionSupported(type, permission);
}

bool Device::isStreamSupported(const LidarStreamType &type) {
    return impl_->isStreamSupported(type);
}

LidarType Device::getType() { return impl_->getType(); }

std::string Device::getName() { return impl_->getName(); }

bool Device::isNetworkReachable() const { return impl_->isNetworkReachable(); }

std::shared_ptr<Frame> Device::waitForFrame(
    const std::chrono::milliseconds &timeout) const {
    return impl_->waitForFrame(timeout);
}

std::shared_ptr<Device> DeviceFactory::create(const std::string &config_file) {
    auto device_config = std::make_shared<DeviceConfig>(config_file);
    return create(device_config);
}

std::shared_ptr<Device> DeviceFactory::create(
    std::shared_ptr<DeviceConfig> config) {
    auto device_impl = std::make_unique<detail::DeviceImpl>(config);
    auto device = std::make_shared<Device>(std::move(device_impl));
    return device;
}

}  // namespace ob_lidar
