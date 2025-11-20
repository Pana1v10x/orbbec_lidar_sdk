#pragma once

#include <string>

#include "option.hpp"
#include "visibility.hpp"

namespace ob_lidar {

template <LidarOption>
struct LidarOptionTrait;

#define DEFINE_LIDAR_OPTION_TRAIT(option, option_type) \
    template <>                                        \
    struct LidarOptionTrait<option> {                  \
        using type = option_type;                      \
    };

// define the type of each option
// FIXME: some options type is not clear
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_IP_ADDRESS, uint32_t)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_PORT, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_MAC_ADDRESS, std::string)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_SUBNET_MASK,uint32_t)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_SCAN_SPEED, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_SCAN_DIRECTION, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_TRANSFER_PROTOCOL, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_WORK_MODE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_INITIATE_DEVICE_CONNECTION, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_SERIAL_NUMBER, std::string)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_REBOOT, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_FACTORY_MODE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_ECHO_MODE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_APPLY_CONFIGS, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_ENABLE_STREAMING, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_FILTER_LEVEL, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_START_MCU_UPGRADE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_END_MCU_UPGRADE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_PROJECT_ID_VERIFICATION, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_PRODUCT_ID_VERIFICATION, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_SEND_MD5_VALUE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_VERIFY_MD5_VALUE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_TRANSFER_FIRMWARE_UPGRADE_PACKAGE,
                          int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_START_FPGA_UPGRADE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_END_FPGA_UPGRADE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_TRANSFER_FPGA_UPGRADE_PACKAGE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_START_MEMS_UPGRADE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_END_MEMS_UPGRADE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_MEMS_ID_VERIFICATION, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_SEND_MEMS_MD5_VALUE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_VERIFY_MEMS_MD5_VALUE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_TRANSFER_MEMS_UPGRADE_PACKAGE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_PRODUCT_MODEL, std::string)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_FIRMWARE_VERSION, std::string)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_FPGA_VERSION, std::string)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_FPGA_VERSION_DATE, std::string)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_HIGH_VOLTAGE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_APD_TEMPERATURE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_TX_VOLTAGE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_SPECIAL_MODE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_SPIN_SPEED, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_MCU_TEMPERATURE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_FPGA_TEMPERATURE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_WARNING_INFO, int)


// Option size trait
template <LidarOption>
struct LidarOptionSizeTrait;

#define DEFINE_LIDAR_OPTION_SIZE_TRAIT(option, option_size) \
    template <>                                             \
    struct LidarOptionSizeTrait<option> {                   \
        static constexpr size_t size = option_size;         \
    };

// serial number 16 bytes
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_SERIAL_NUMBER, 16)
// firmware version 16 bytes
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_FIRMWARE_VERSION, 16)
// fpga version 16 bytes
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_FPGA_VERSION, 16)
// mac address 8 bytes,use first 6 bytes
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_MAC_ADDRESS, 8)

DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_PRODUCT_MODEL, 16)

DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_IP_ADDRESS, sizeof(uint32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_PORT, sizeof(uint32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_SUBNET_MASK, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_SCAN_SPEED, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_SCAN_DIRECTION, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_TRANSFER_PROTOCOL,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_WORK_MODE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_INITIATE_DEVICE_CONNECTION,
                               sizeof(int32_t))

DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_REBOOT, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_FACTORY_MODE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_ECHO_MODE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_APPLY_CONFIGS, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_ENABLE_STREAMING,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_FILTER_LEVEL, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_START_MCU_UPGRADE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_END_MCU_UPGRADE, sizeof(int32_t))
// FIXME: the size of the following options size is not clear
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_PROJECT_ID_VERIFICATION,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_PRODUCT_ID_VERIFICATION,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_SEND_MD5_VALUE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_VERIFY_MD5_VALUE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(
    OB_LIDAR_OPTION_TRANSFER_FIRMWARE_UPGRADE_PACKAGE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_START_FPGA_UPGRADE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_END_FPGA_UPGRADE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_TRANSFER_FPGA_UPGRADE_PACKAGE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_START_MEMS_UPGRADE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_END_MEMS_UPGRADE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_MEMS_ID_VERIFICATION,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_SEND_MEMS_MD5_VALUE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_VERIFY_MEMS_MD5_VALUE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_TRANSFER_MEMS_UPGRADE_PACKAGE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_HIGH_VOLTAGE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_APD_TEMPERATURE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_TX_VOLTAGE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_SPIN_SPEED, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_MCU_TEMPERATURE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_FPGA_TEMPERATURE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_SPECIAL_MODE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_WARNING_INFO, sizeof(int32_t))


class Device;

OB_EXPORT uint32_t ipAddrToInt(const std::string& ip_address);

OB_EXPORT std::string intToIpAddr(uint32_t ip_address);

// Provide some helper functions to manage device settings, demonstrating the
// usage of one option, but it's not necessary to show all the options

/**
 * @brief Sets the IP address for the specified device.
 *
 * This function configures the IP address of the given device to the provided
 * IP address string.
 *
 * @param device A shared pointer to the Device object whose IP address is to
 * be set.
 * @param ip_address A string representing the new IP address to be assigned
 * to the device.
 * @return Status indicating the success or failure of the operation.
 */
OB_EXPORT Status setIPAddress(std::shared_ptr<Device> device,
                              const std::string& ip_address);

/**
 * @brief Retrieves the IP address of the specified device.
 *
 * This function takes a shared pointer to a Device object and returns its IP
 * address as a string.
 *
 * @param device A shared pointer to the Device object whose IP address is to be
 * retrieved.
 * @return std::string The IP address of the specified device.
 */
OB_EXPORT std::string getIPAddress(std::shared_ptr<Device> device);

/**
 * @brief Sets the port for the given device.
 *
 * This function configures the specified device to use the provided port
 * number.
 *
 * @param device A shared pointer to the Device object that needs to be
 * configured.
 * @param port The port number to set for the device.
 * @return Status indicating the success or failure of the operation.
 */
OB_EXPORT Status setPort(std::shared_ptr<Device> device, int port);

/**
 * @brief Retrieves the port number associated with the given device.
 *
 * This function takes a shared pointer to a Device object and returns the port
 * number that the device is connected to.
 *
 * @param device A shared pointer to the Device object for which the port number
 * is to be retrieved.
 * @return int The port number associated with the given device.
 */
OB_EXPORT int getPort(std::shared_ptr<Device> device);

/**
 * @brief Sets the MAC address for the specified device.
 *
 * This function configures the MAC address of the given device to the provided
 * MAC address string.
 *
 * @param device A shared pointer to the Device object whose MAC address is to
 * be set.
 * @param mac_address A string representing the new MAC address to be assigned
 * to the device.
 * @return Status indicating the success or failure of the operation.
 */
OB_EXPORT Status setMACAddress(std::shared_ptr<Device> device,
                               const std::string& mac_address);

/**
 * @brief Retrieves the MAC address of the specified device.
 *
 * This function queries the given device to obtain its MAC address.
 *
 * @param device A shared pointer to the Device object from which the MAC
 * address is to be retrieved.
 * @return A string containing the MAC address of the device.
 */
OB_EXPORT std::string getMACAddress(std::shared_ptr<Device> device);

/**
 * @brief Sets the subnet mask for the specified device.
 *
 * This function configures the subnet mask for the given device, which is
 * essential for network configuration and communication.
 *
 * @param device A shared pointer to the Device object for which the subnet
 *               mask is to be set.
 * @param subnet_mask A string representing the subnet mask to be set for the
 *                    device (e.g., "255.255.255.0").
 * @return Status indicating the success or failure of the operation.
 */
OB_EXPORT Status setSubnetMask(std::shared_ptr<Device> device,
                               const std::string& subnet_mask);

/**
 * @brief Retrieves the subnet mask of the specified device.
 *
 * This function queries the given device to obtain its subnet mask,
 * which is a crucial part of the device's network configuration.
 *
 * @param device A shared pointer to the Device object for which the subnet mask
 * is to be retrieved.
 * @return A string representing the subnet mask of the specified device.
 */
OB_EXPORT std::string getSubnetMask(std::shared_ptr<Device> device);

/**
 * @brief Sets the scan speed for the given device.
 *
 * This function configures the scan speed of the specified device.
 *
 * @param device A shared pointer to the Device object for which the scan speed
 * is to be set.
 * @param scan_speed An integer representing the desired scan speed.
 * @return Status indicating the success or failure of the operation.
 */
OB_EXPORT Status setScanSpeed(std::shared_ptr<Device> device, int scan_speed);

/**
 * @brief Retrieves the scan speed of the specified device.
 *
 * This function queries the given device to determine its current scan speed.
 *
 * @param device A shared pointer to the Device object from which to retrieve
 * the scan speed.
 * @return An integer representing the scan speed of the device.
 */
OB_EXPORT int getScanSpeed(std::shared_ptr<Device> device);

/**
 * @brief Retrieves the real-time motor spin speed of the specified device.
 *
 * This function returns the current spin speed of the motor in the device,
 * which is different from the scan speed. The spin speed refers to the
 * actual rotational speed of the motor in real-time.
 *
 * @param device A shared pointer to the Device object for which the spin speed is to be retrieved.
 * @return The current spin speed of the motor.
 */
OB_EXPORT int getSpinSpeed(std::shared_ptr<Device> device);

/**
 * @brief Sets the scan direction for the specified device.
 *
 * This function configures the scan direction of the given device.
 *
 * @param device A shared pointer to the Device object for which the scan
 * direction is to be set.
 * @param scan_direction An integer representing the desired scan direction. The
 * specific values and their meanings should be defined in the device
 * documentation.
 * @return Status indicating the success or failure of the operation.
 */
OB_EXPORT Status setScanDirection(std::shared_ptr<Device> device,
                                  int scan_direction);

/**
 * @brief Retrieves the scan direction of the specified device.
 *
 * This function queries the given device to determine the direction
 * in which it performs scanning operations.
 *
 * @param device A shared pointer to the Device object for which the scan
 * direction is to be retrieved.
 * @return An integer representing the scan direction of the device.
 */
OB_EXPORT int getScanDirection(std::shared_ptr<Device> device);

/**
 * @brief Sets the transfer protocol for the specified device.
 *
 * This function configures the transfer protocol to be used by the given
 * device.
 *
 * @param device A shared pointer to the Device object for which the transfer
 * protocol is to be set.
 * @param transfer_protocol An integer representing the transfer protocol to be
 * set.
 * @return Status indicating the success or failure of the operation.
 */
OB_EXPORT Status setTransferProtocol(std::shared_ptr<Device> device,
                                     int transfer_protocol);

/**
 * @brief Retrieves the transfer protocol used by the specified device.
 *
 * This function returns an integer representing the transfer protocol
 * used by the given device. The transfer protocol determines how data
 * is communicated between the device and the host system.
 *
 * @param device A shared pointer to the Device object for which the transfer
 * protocol is to be retrieved.
 * @return An integer representing the transfer protocol of the device.
 */
OB_EXPORT int getTransferProtocol(std::shared_ptr<Device> device);

/**
 * @brief Sets the work mode of the specified device.
 *
 * This function configures the device to operate in the specified work mode.
 *
 * @param device A shared pointer to the Device object that needs to be
 * configured.
 * @param work_mode An integer representing the desired work mode.
 * @return Status indicating the success or failure of the operation.
 */
OB_EXPORT Status setWorkMode(std::shared_ptr<Device> device, int work_mode);

/**
 * @brief Retrieves the current work mode of the specified device.
 *
 * This function queries the given device to determine its current operational
 * mode.
 *
 * @param device A shared pointer to the Device object whose work mode is to be
 * retrieved.
 * @return An integer representing the current work mode of the device.
 */
OB_EXPORT int getWorkMode(std::shared_ptr<Device> device);

/**
 * @brief Reboots the specified device.
 *
 * This function initiates a reboot sequence on the provided device.
 *
 * @param device A shared pointer to the Device object that needs to be
 * rebooted.
 * @param reboot An integer flag indicating the type of reboot to perform.
 * @return Status indicating the success or failure of the operation.
 */
OB_EXPORT Status reboot(std::shared_ptr<Device> device, int reboot);

/**
 * @brief Reboots the specified device.
 *
 * This function initiates a reboot sequence on the provided device.
 *
 * @param device A shared pointer to the Device object that needs to be
 * rebooted.
 * @return Status indicating the success or failure of the operation.
 */
OB_EXPORT Status reboot(std::shared_ptr<Device> device);
/**
 * @brief Sets the echo mode for the specified device.
 *
 * This function configures the echo mode of the given device. The echo mode
 * determines how the device handles multiple return signals.
 *
 * @param device A shared pointer to the Device object for which the echo mode
 * is to be set.
 * @param echo_mode An integer representing the desired echo mode. The specific
 * values and their meanings should be defined in the device's documentation.
 * @return Status indicating the success or failure of the operation.
 */
OB_EXPORT Status setEchoMode(std::shared_ptr<Device> device, int echo_mode);

/**
 * @brief Retrieves the echo mode of the specified device.
 *
 * This function queries the given device to determine its current echo mode.
 * The echo mode typically refers to the way the device handles multiple return
 * signals from a single laser pulse, which can be useful in various lidar
 * applications.
 *
 * @param device A shared pointer to the Device object for which the echo mode
 * is to be retrieved.
 * @return An integer representing the echo mode of the device.
 */
OB_EXPORT int getEchoMode(std::shared_ptr<Device> device);

/**
 * @brief Sets the configuration application status for the specified device.
 *
 * This function updates the configuration application status of the given
 * device.
 *
 * @param device A shared pointer to the Device object for which the
 * configuration status is to be set.
 * @param apply_configs An integer representing the configuration application
 * status to be set.
 * @return Status indicating the success or failure of the operation.
 */
OB_EXPORT Status setApplyConfigs(std::shared_ptr<Device> device,
                                 int apply_configs);

/**
 * @brief Applies the given configurations to the specified device.
 *
 * This function sets the configurations on the device so that they
 * persist even after a reboot.
 *
 * @param device A shared pointer to the Device object to which the
 * configurations will be applied.
 * @return Status indicating the success or failure of the operation.
 */
OB_EXPORT Status applyConfigs(std::shared_ptr<Device> device);

/**
 * @brief Sets the streaming state of the specified device.
 *
 * This function enables or disables streaming for the given device.
 *
 * @param device A shared pointer to the Device object for which the streaming
 * state is to be set.
 * @param enable_streaming An integer flag indicating whether to enable or
 * disable streaming. A non-zero value enables streaming, while zero disables
 * it.
 * @return Status indicating the success or failure of the operation.
 */
OB_EXPORT Status setEnableStreaming(std::shared_ptr<Device> device,
                                    int enable_streaming);

/**
 * @brief Retrieves the current streaming state of the specified device.
 *
 * This function queries the given device to determine whether it is currently
 * streaming data.
 */
OB_EXPORT int getEnableStreaming(std::shared_ptr<Device> device);

/**
 * @brief Retrieves the firmware version of the specified device.
 *
 * This function queries the given device to obtain its firmware version.
 *
 * @param device A shared pointer to the Device object for which the firmware
 * version is to be retrieved.
 * @return A string representing the firmware version of the device.
 */
OB_EXPORT std::string getFirmwareVersion(std::shared_ptr<Device> device);

/**
 * @brief Retrieves the FPGA version from the specified device.
 *
 * This function queries the given device to obtain the version of the FPGA
 * firmware currently in use.
 *
 * @param device A shared pointer to the Device object from which to retrieve
 * the FPGA version.
 * @return A string representing the FPGA version.
 */
OB_EXPORT std::string getFPGAVersion(std::shared_ptr<Device> device);

/**
 * @brief Retrieves the product model of the specified device.
 *
 * This function queries the given device to obtain its product model
 * identifier.
 *
 * @param device A shared pointer to the Device object for which the product
 * model is to be retrieved.
 * @return An string representing the product model of the device.
 */
OB_EXPORT std::string getProductModel(std::shared_ptr<Device> device);

/**
 * @brief Retrieves the serial number of the specified device.
 *
 * This function queries the given device to obtain its serial number.
 *
 * @param device A shared pointer to the Device object for which the serial
 * number is to be retrieved.
 * @return A string representing the serial number of the device.
 */
OB_EXPORT std::string getSerialNumber(std::shared_ptr<Device> device);

/**
 * @brief Sets the serial number for the specified device.
 *
 * This function assigns a serial number to the given device.
 *
 * @param device A shared pointer to the Device object for which the serial
 * number is to be set.
 * @param serial_number A string representing the serial number to be assigned
 * to the device.
 * @return Status indicating the success or failure of the operation.
 */
OB_EXPORT Status setSerialNumber(std::shared_ptr<Device> device,
                                 const std::string& serial_number);

/**
 * @brief Initiates a connection to the specified device.
 *
 * This function attempts to establish a connection with the given device.
 *
 * @param device A shared pointer to the Device object to connect to.
 * @return Status indicating the success or failure of the connection attempt.
 */
OB_EXPORT Status initiateConnection(std::shared_ptr<Device> device);

/**
 * @brief Retrieves the spatial mode of the specified device.
 *
 * This function queries the given device to determine its current spatial mode.
 *
 * @param device A shared pointer to the Device object for which the spatial mode is to be retrieved.
 * @return An integer representing the spatial mode of the device.
 */
OB_EXPORT int getSpacialMode(std::shared_ptr<Device> device);

/**
 * @brief Sets the spatial mode for the specified device.
 *
 * This function configures the spatial mode of the given device.
 *
 * @param device A shared pointer to the Device object for which the spatial mode is to be set.
 * @param spacial_mode An integer representing the desired spatial mode.
 * @return Status indicating the success or failure of the operation.
 */
OB_EXPORT Status setSpacialMode(std::shared_ptr<Device> device,
                                int spacial_mode);

/**
 * @brief Retrieves the warning information from the specified device.
 *
 * This function queries the given device to obtain any warning information
 * that it may have. The warning information can be used to diagnose issues
 * or monitor the device's status.
 *
 * @param device A shared pointer to the Device object from which the warning
 * information is to be retrieved.
 * @return An integer representing the warning information of the device.
 */
OB_EXPORT int getWarningInfo(std::shared_ptr<Device> device);

/**
 * @brief Sets the filter level for the specified device.
 *
 * This function configures the filter level of the given device.
 */

OB_EXPORT Status setFilterLevel(std::shared_ptr<Device> device, int filter_level);

/**
 * @brief Retrieves the filter level of the specified device.
 *
 * This function queries the given device to determine its current filter level.
 * The filter level is an integer value that represents the degree of filtering
 * applied by the device.
 *
 * @param device A shared pointer to the Device object for which the filter level
 *               is to be retrieved.
 * @return An integer representing the filter level of the device.
 */
OB_EXPORT int getFilterLevel(std::shared_ptr<Device> device);

/**
 * @brief Converts a LidarOption enum value to its corresponding string literal.
 *
 * This function takes a LidarOption enum value and returns its string
 * representation. It is useful for logging, debugging, or displaying the option
 * in a human-readable format.
 *
 * @param option The LidarOption enum value to be converted.
 * @return A std::string containing the string literal representation of the
 * given LidarOption.
 */

OB_EXPORT std::string asStringLiteral(LidarOption option);

}  // namespace ob_lidar
