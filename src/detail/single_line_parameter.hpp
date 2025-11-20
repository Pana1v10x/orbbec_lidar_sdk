#pragma once

#include "parameter.hpp"

namespace ob_lidar::detail {
class SingleLineParameter : public LidarParameter {
   public:
    SingleLineParameter();
    ~SingleLineParameter() override;
};

}  // namespace ob_lidar::detail
