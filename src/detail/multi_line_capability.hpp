#pragma once

#include "capability.hpp"

namespace ob_lidar {
class MultiLineCapabilities : public LidarCapabilitiesInterface {
   public:
    MultiLineCapabilities();

    ~MultiLineCapabilities() override = default;
};
}  // namespace ob_lidar
