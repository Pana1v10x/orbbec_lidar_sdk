#pragma once

#include <memory>
#include <orbbec_lidar/option.hpp>
#include <string>
#include <unordered_map>

namespace ob_lidar {

#define REGISTER_OPTION_PERMISSION(option, permission) \
    do {                                               \
        supported_options_map_[option] = permission;   \
    } while (0)

#define REGISTER_STREAM(stream, supported)         \
    do {                                           \
        supported_stream_map_[stream] = supported; \
    } while (0)

class LidarCapabilitiesInterface {
   public:
    LidarCapabilitiesInterface() = default;

    virtual ~LidarCapabilitiesInterface() = default;

    bool isOptionSupported(const LidarOption &option) const;

    bool isOptionSupported(const LidarOption &type,
                           const LidarOptionPermission &permission) const;

    bool isStreamSupported(const LidarStreamType &type) const;

    bool isStreamFreqSupported(const LidarStreamType &type,
                               const double &freq) const;

   protected:
    std::unordered_map<LidarOption, LidarOptionPermission>
        supported_options_map_;
    std::unordered_map<LidarStreamType, bool> supported_stream_map_;
    std::unordered_map<LidarStreamType, std::vector<double>>
        supported_stream_freq_map_;
};
}  // namespace ob_lidar
