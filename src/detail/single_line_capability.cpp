#include "single_line_capability.hpp"

#include "../logger.hpp"

namespace ob_lidar {

SingleLineCapabilities::SingleLineCapabilities()
    : LidarCapabilitiesInterface() {
    // IP address, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_IP_ADDRESS, READ_WRITE);
    // port, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_PORT, READ_WRITE);
    // mac address, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_MAC_ADDRESS, READ_WRITE);
    // subnet mask, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SUBNET_MASK, READ_WRITE);
    // scan speed, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SCAN_SPEED, READ_WRITE);
    // scan direction, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SCAN_DIRECTION, READ_WRITE);
    // transfer protocol, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_TRANSFER_PROTOCOL, READ_WRITE);
    // work mode, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_WORK_MODE, READ_WRITE);
    // initiate device connection, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_INITIATE_DEVICE_CONNECTION,
                               WRITE);
    // serial number, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SERIAL_NUMBER, READ_WRITE);
    // reboot, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_REBOOT, READ_WRITE);
    // factory mode, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FACTORY_MODE, READ_WRITE);
    // echo mode, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_ECHO_MODE, READ_WRITE);
    // apply configs, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_APPLY_CONFIGS, WRITE);
    // enable streaming, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_ENABLE_STREAMING, READ_WRITE);
    // filter level, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FILTER_LEVEL, READ_WRITE);
    // product model, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_PRODUCT_MODEL, READ);
    // firmware version, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FIRMWARE_VERSION, READ);
    // FPGA version, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FPGA_VERSION, READ_WRITE);
    // spin speed, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SPIN_SPEED, READ);
    // MCU temperature, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_MCU_TEMPERATURE, READ);
    // FPGA temperature, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FPGA_TEMPERATURE, READ);
    // FPGA version date, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FPGA_VERSION_DATE, READ);
    // high voltage, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_HIGH_VOLTAGE, READ);
    // APD temperature, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_APD_TEMPERATURE, READ);
    // TX voltage, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_TX_VOLTAGE, READ);
    // filter level, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FILTER_LEVEL, READ_WRITE);
    // warning info, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_WARNING_INFO, READ);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SPECIAL_MODE, READ_WRITE);
    // imu, not supported
    REGISTER_STREAM(LidarStreamType::IMU, false);
    // point cloud, not supported
    REGISTER_STREAM(LidarStreamType::POINT_CLOUD, false);
    // sphere point cloud, not supported
    REGISTER_STREAM(LidarStreamType::SPHERE_POINT_CLOUD, false);
    // scan data, supported
    REGISTER_STREAM(LidarStreamType::SCAN, true);
    // single-line lidar supports 15hz, 20hz, 25hz, 30hz
    supported_stream_freq_map_[LidarStreamType::SCAN] = {15.0, 20.0, 25.0,
                                                         30.0};
}

}  // namespace ob_lidar
