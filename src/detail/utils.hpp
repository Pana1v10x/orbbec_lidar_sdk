#pragma once
#include <string>

#include "types.hpp"

namespace ob_lidar {
std::string asStringLiteral(const LidarCommandInterface& command_id);

std::string asStringLiteral(Status status);

std::string asStringLiteral(LidarType type);

uint8_t calcCrc8(const uint8_t *p, uint8_t len);

}  // namespace ob_lidar
