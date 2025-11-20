#pragma once

#include "parameter.hpp"

namespace ob_lidar::detail {
class MultiLineParameter : public LidarParameter {
   public:
    MultiLineParameter();
    ~MultiLineParameter() override;
};
}  // namespace ob_lidar::detail
