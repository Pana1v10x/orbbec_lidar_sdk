#include "capability.hpp"

#include "../logger.hpp"

namespace ob_lidar {
bool LidarCapabilitiesInterface::isOptionSupported(
    const LidarOption &option) const {
    return supported_options_map_.find(option) != supported_options_map_.end();
}

bool LidarCapabilitiesInterface::isOptionSupported(
    const LidarOption &type, const LidarOptionPermission &permission) const {
    const auto it = supported_options_map_.find(type);
    if (it == supported_options_map_.end()) {
        return false;
    }
    if (permission == READ) {
        return it->second == READ || it->second == READ_WRITE;
    }
    if (permission == WRITE) {
        return it->second == WRITE || it->second == READ_WRITE;
    }
    return it->second == permission;
}

bool LidarCapabilitiesInterface::isStreamSupported(
    const LidarStreamType &type) const {
    return supported_stream_map_.find(type) != supported_stream_map_.end();
}

bool LidarCapabilitiesInterface::isStreamFreqSupported(
    const LidarStreamType &type, const double &freq) const {
    if (!isStreamSupported(type)) {
        LOG_ERROR("Stream type {} is not supported", static_cast<int>(type));
        return false;
    }
    auto freq_list = supported_stream_freq_map_.at(type);
    return std::find(freq_list.begin(), freq_list.end(), freq) !=
           freq_list.end();
}

}  // namespace ob_lidar
