#include "parameter.hpp"

namespace ob_lidar::detail {
LidarParameter::LidarParameter() = default;

LidarParameter::~LidarParameter() = default;

bool LidarParameter::hasParameter(const std::string &key) {
    return int_parameters_.count(key) || double_parameters_.count(key) ||
           string_parameters_.count(key);
}
}  // namespace ob_lidar::detail
