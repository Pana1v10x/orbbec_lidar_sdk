#pragma once

#include "capability.hpp"

namespace ob_lidar {
class SingleLineCapabilities : public LidarCapabilitiesInterface {
   public:
    SingleLineCapabilities();

    ~SingleLineCapabilities() override = default;
};
}  // namespace ob_lidar
