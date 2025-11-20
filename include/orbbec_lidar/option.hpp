#pragma once
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "types.hpp"

namespace ob_lidar {

/**
 * @brief Enumeration of various Lidar options. you can also call it as
 * Command for the Lidar device.
 */

enum LidarOption {
    // IP address
    OB_LIDAR_OPTION_IP_ADDRESS,
    // Port
    OB_LIDAR_OPTION_PORT,
    // MAC address
    OB_LIDAR_OPTION_MAC_ADDRESS,
    // Subnet mask
    OB_LIDAR_OPTION_SUBNET_MASK,
    // Scan speed, unit: RPM, scan speed means the expected motor speed
    OB_LIDAR_OPTION_SCAN_SPEED,
    // Scan direction
    OB_LIDAR_OPTION_SCAN_DIRECTION,
    // Transfer Protocol (TCP/UDP)
    OB_LIDAR_OPTION_TRANSFER_PROTOCOL,
    // work mode
    OB_LIDAR_OPTION_WORK_MODE,
    // Initiate device connection
    OB_LIDAR_OPTION_INITIATE_DEVICE_CONNECTION,
    //  serial number
    OB_LIDAR_OPTION_SERIAL_NUMBER,
    // reboot device
    OB_LIDAR_OPTION_REBOOT,
    // enter factory mode
    OB_LIDAR_OPTION_FACTORY_MODE,
    // echo mode
    OB_LIDAR_OPTION_ECHO_MODE,
    // Saves the current LiDAR configuration to flash memory.
    // On the next power cycle, the device will use these saved settings.
    OB_LIDAR_OPTION_APPLY_CONFIGS,
    // enable or disable streaming, 0: disable, 1: enable
    OB_LIDAR_OPTION_ENABLE_STREAMING,
    // filter level， 0-10.
    OB_LIDAR_OPTION_FILTER_LEVEL,
    // start mcu upgrade
    OB_LIDAR_OPTION_START_MCU_UPGRADE,
    // end mcu upgrade
    OB_LIDAR_OPTION_END_MCU_UPGRADE,
    // Project ID verification
    OB_LIDAR_OPTION_PROJECT_ID_VERIFICATION,
    // Product ID verification
    OB_LIDAR_OPTION_PRODUCT_ID_VERIFICATION,
    // send md5 value
    OB_LIDAR_OPTION_SEND_MD5_VALUE,
    // verify md5 value
    OB_LIDAR_OPTION_VERIFY_MD5_VALUE,
    // transfer firmware upgrade package
    OB_LIDAR_OPTION_TRANSFER_FIRMWARE_UPGRADE_PACKAGE,
    // start fpga upgrade
    OB_LIDAR_OPTION_START_FPGA_UPGRADE,
    // end fpga upgrade
    OB_LIDAR_OPTION_END_FPGA_UPGRADE,
    // transfer fpga upgrade package
    OB_LIDAR_OPTION_TRANSFER_FPGA_UPGRADE_PACKAGE,
    // start MEMS upgrade
    OB_LIDAR_OPTION_START_MEMS_UPGRADE,
    // end MEMS upgrade
    OB_LIDAR_OPTION_END_MEMS_UPGRADE,
    // MEMS ID verification
    OB_LIDAR_OPTION_MEMS_ID_VERIFICATION,
    // send MEMS md5 value
    OB_LIDAR_OPTION_SEND_MEMS_MD5_VALUE,
    // verify MEMS md5 value
    OB_LIDAR_OPTION_VERIFY_MEMS_MD5_VALUE,
    // transfer MEMS upgrade package
    OB_LIDAR_OPTION_TRANSFER_MEMS_UPGRADE_PACKAGE,
    // product model
    OB_LIDAR_OPTION_PRODUCT_MODEL,
    // firmware version
    OB_LIDAR_OPTION_FIRMWARE_VERSION,
    // FPGA version
    OB_LIDAR_OPTION_FPGA_VERSION,
    // spin speed, spin speed means the real time motor speed, read only
    OB_LIDAR_OPTION_SPIN_SPEED,
    // MCU temperature
    OB_LIDAR_OPTION_MCU_TEMPERATURE,
    // FPGA temperature
    OB_LIDAR_OPTION_FPGA_TEMPERATURE,
    // FPGA version date
    OB_LIDAR_OPTION_FPGA_VERSION_DATE,
    // high voltage
    OB_LIDAR_OPTION_HIGH_VOLTAGE,
    // special mode
    OB_LIDAR_OPTION_SPECIAL_MODE,
    // APD temperature
    OB_LIDAR_OPTION_APD_TEMPERATURE,
    // TX voltage
    OB_LIDAR_OPTION_TX_VOLTAGE,
    // warning info
    OB_LIDAR_OPTION_WARNING_INFO,
    OB_LIDAR_OPTION_COUNT,
};

/**
 * @brief Permission for the Lidar option.
 */
enum LidarOptionPermission {
    // read
    READ = 0,
    // write
    WRITE = 1,
    // read and write
    READ_WRITE = 2
};

/**
 * @brief Operation for the Lidar option.
 */
enum class LidarOptionOp {
    GET,  // get the value of the option
    SET,  // set the value of the option
};

}  // namespace ob_lidar
