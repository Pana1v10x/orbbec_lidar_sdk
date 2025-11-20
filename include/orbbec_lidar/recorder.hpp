#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "device.hpp"
#include "visibility.hpp"
namespace ob_lidar {
namespace detail {
class RecorderImpl;
}  // namespace detail

class OB_EXPORT Recorder {
   public:
    /**
     * @class Recorder
     * @brief The Recorder class is responsible for recording data from a Lidar
     * device.
     *
     * The Recorder class provides functionality to record data from a Lidar
     * device and save it to a specified output directory.
     */
    Recorder(std::shared_ptr<Device> device, const std::string &output_dir);

    ~Recorder();

    /**
     * @brief Start recording data from the Lidar device.
     */
    void start() const;

    /**
     * @brief Stop recording data from the Lidar device.
     */
    void stop()const;

    /**
     * @brief Enable recording  data from the Lidar device.
     *
     * @param enable Whether to enable recording IMU data.
     */
    void enableRecord(bool enable) const;

   private:
    std::unique_ptr<detail::RecorderImpl> impl_;
};
}  // namespace ob_lidar
