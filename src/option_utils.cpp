#include "orbbec_lidar/option_utils.hpp"

#include <sstream>
#include <stdexcept>

#include "./logger.hpp"
#include "orbbec_lidar/device.hpp"
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#elif __linux__
#include <arpa/inet.h>
#endif

namespace ob_lidar {
// some helper functions, show the usage of the option, not all

uint32_t ipAddrToInt(const std::string& ip_address) {
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        throw std::runtime_error("Failed to initialize Winsock");
    }
#endif

    struct in_addr addr {};

    if (inet_pton(AF_INET, ip_address.c_str(), &addr) != 1) {
#ifdef _WIN32
        WSACleanup();  // clean Winsock
#endif
        throw std::invalid_argument("Invalid IP address");
    }

#ifdef _WIN32
    WSACleanup();  // clean Winsock
#endif

    return addr.s_addr;
}

std::string intToIpAddr(uint32_t ip_address) {
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        throw std::runtime_error("Failed to initialize Winsock");
    }
#endif

    struct in_addr addr {};
    addr.s_addr = ip_address;

    char str[INET_ADDRSTRLEN];

    // using inet_ntop instead of inet_ntoa
    if (inet_ntop(AF_INET, &addr, str, INET_ADDRSTRLEN) == nullptr) {
#ifdef _WIN32
        WSACleanup();  // cleanup Winsock
#endif
        throw std::runtime_error("Failed to convert IP address");
    }

#ifdef _WIN32
    WSACleanup();  // cleanup
#endif

    return std::string(str);
}

Status setIPAddress(std::shared_ptr<Device> device,
                    const std::string& ip_address) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_IP_ADDRESS, WRITE)) {
        LOG_ERROR("set IP address is not supported");
        return Status::ERR;
    }
    auto ip = ipAddrToInt(ip_address);
    return device->setOption<OB_LIDAR_OPTION_IP_ADDRESS>(ip);
}

std::string getIPAddress(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_IP_ADDRESS, READ)) {
        LOG_ERROR("get IP address is not supported");
        return "";
    }
    const auto ip_addr = device->getOption<OB_LIDAR_OPTION_IP_ADDRESS>();
    return intToIpAddr(ip_addr);
}

Status setPort(std::shared_ptr<Device> device, int port) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_PORT, WRITE)) {
        LOG_ERROR("set port is not supported");
        return Status::ERR;
    }
    return device->setOption<OB_LIDAR_OPTION_PORT>(port);
}

int getPort(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_PORT, READ)) {
        LOG_ERROR("get port is not supported");
        return -1;
    }
    return device->getOption<OB_LIDAR_OPTION_PORT>();
}

Status setMACAddress(std::shared_ptr<Device> device,
                     const std::string& mac_address) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_MAC_ADDRESS, WRITE)) {
        LOG_ERROR("set MAC address is not supported");
        return Status::ERR;
    }
    return device->setOption<OB_LIDAR_OPTION_MAC_ADDRESS>(mac_address);
}

std::string getMACAddress(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_MAC_ADDRESS, READ)) {
        LOG_ERROR("get MAC address is not supported");
        return "";
    }
    return device->getOption<OB_LIDAR_OPTION_MAC_ADDRESS>();
}

Status setSubnetMask(std::shared_ptr<Device> device,
                     const std::string& subnet_mask) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_SUBNET_MASK, WRITE)) {
        LOG_ERROR("set subnet mask is not supported");
        return Status::ERR;
    }
    const auto int_subnet_mask = ipAddrToInt(subnet_mask);
    return device->setOption<OB_LIDAR_OPTION_SUBNET_MASK>(int_subnet_mask);
}

std::string getSubnetMask(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_SUBNET_MASK, READ)) {
        LOG_ERROR("get subnet mask is not supported");
        return "";
    }
    auto sub_netmask = device->getOption<OB_LIDAR_OPTION_SUBNET_MASK>();
    return intToIpAddr(sub_netmask);
}

Status setScanSpeed(std::shared_ptr<Device> device, int scan_speed) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_SCAN_SPEED, WRITE)) {
        LOG_ERROR("set scan speed is not supported");
        return Status::ERR;
    }
    return device->setOption<OB_LIDAR_OPTION_SCAN_SPEED>(scan_speed * 60);
}

int getScanSpeed(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_SCAN_SPEED, READ)) {
        LOG_ERROR("get scan speed is not supported");
        return -1;
    }
    return device->getOption<OB_LIDAR_OPTION_SCAN_SPEED>() / 60;
}

int getSpinSpeed(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_SPIN_SPEED, READ)) {
        LOG_ERROR("get spin speed is not supported");
        return -1;
    }
    return device->getOption<OB_LIDAR_OPTION_SPIN_SPEED>();
}

Status setScanDirection(std::shared_ptr<Device> device, int scan_direction) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_SCAN_DIRECTION, WRITE)) {
        LOG_ERROR("set scan direction is not supported");
        return Status::ERR;
    }
    return device->setOption<OB_LIDAR_OPTION_SCAN_DIRECTION>(scan_direction);
}

int getScanDirection(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_SCAN_DIRECTION, READ)) {
        LOG_ERROR("get scan direction is not supported");
        return -1;
    }
    return device->getOption<OB_LIDAR_OPTION_SCAN_DIRECTION>();
}

Status setTransferProtocol(std::shared_ptr<Device> device,
                           int transfer_protocol) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_TRANSFER_PROTOCOL, WRITE)) {
        LOG_ERROR("set transfer protocol is not supported");
        return Status::ERR;
    }
    return device->setOption<OB_LIDAR_OPTION_TRANSFER_PROTOCOL>(
        transfer_protocol);
}

int getTransferProtocol(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_TRANSFER_PROTOCOL, READ)) {
        LOG_ERROR("get transfer protocol is not supported");
        return -1;
    }
    return device->getOption<OB_LIDAR_OPTION_TRANSFER_PROTOCOL>();
}

Status setWorkMode(std::shared_ptr<Device> device, int work_mode) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_WORK_MODE, WRITE)) {
        LOG_ERROR("set work mode is not supported");
        return Status::ERR;
    }
    return device->setOption<OB_LIDAR_OPTION_WORK_MODE>(work_mode);
}

int getWorkMode(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_WORK_MODE, READ)) {
        LOG_ERROR("get work mode is not supported");
        return -1;
    }
    return device->getOption<OB_LIDAR_OPTION_WORK_MODE>();
}

Status reboot(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_REBOOT, WRITE)) {
        LOG_ERROR("set reboot is not supported");
        return Status::ERR;
    }
    return device->setOption<OB_LIDAR_OPTION_REBOOT>(1);
}
Status setEchoMode(std::shared_ptr<Device> device, int echo_mode) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_ECHO_MODE, WRITE)) {
        LOG_ERROR("set echo mode is not supported");
        return Status::ERR;
    }
    return device->setOption<OB_LIDAR_OPTION_ECHO_MODE>(echo_mode);
}

int getEchoMode(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_ECHO_MODE, READ)) {
        LOG_ERROR("get echo mode is not supported");
        return -1;
    }
    return device->getOption<OB_LIDAR_OPTION_ECHO_MODE>();
}

Status setApplyConfigs(std::shared_ptr<Device> device, int apply_configs) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_APPLY_CONFIGS, WRITE)) {
        LOG_ERROR("set apply configs is not supported");
        return Status::ERR;
    }
    return device->setOption<OB_LIDAR_OPTION_APPLY_CONFIGS>(apply_configs);
}

Status applyConfigs(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_APPLY_CONFIGS, WRITE)) {
        LOG_ERROR("set apply configs is not supported");
        return Status::ERR;
    }
    return device->setOption<OB_LIDAR_OPTION_APPLY_CONFIGS>(1);
}

Status setEnableStreaming(std::shared_ptr<Device> device,
                          int enable_streaming) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_ENABLE_STREAMING, WRITE)) {
        LOG_ERROR("set enable streaming is not supported");
        return Status::ERR;
    }
    return device->setOption<OB_LIDAR_OPTION_ENABLE_STREAMING>(
        enable_streaming);
}

int getEnableStreaming(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_ENABLE_STREAMING, READ)) {
        LOG_ERROR("get enable streaming is not supported");
        return -1;
    }
    return device->getOption<OB_LIDAR_OPTION_ENABLE_STREAMING>();
}

std::string getFirmwareVersion(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_FIRMWARE_VERSION, READ)) {
        LOG_ERROR("get firmware version is not supported");
        return "";
    }
    return device->getOption<OB_LIDAR_OPTION_FIRMWARE_VERSION>();
}

std::string getFPGAVersion(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_FPGA_VERSION, READ)) {
        LOG_ERROR("get FPGA version is not supported");
        return "";
    }
    return device->getOption<OB_LIDAR_OPTION_FPGA_VERSION>();
}

std::string getProductModel(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_PRODUCT_MODEL, READ)) {
        LOG_ERROR("get product model is not supported");
        return "";
    }
    return device->getOption<OB_LIDAR_OPTION_PRODUCT_MODEL>();
}

std::string getSerialNumber(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_SERIAL_NUMBER, READ)) {
        LOG_ERROR("get serial number is not supported");
        return "";
    }
    return device->getOption<OB_LIDAR_OPTION_SERIAL_NUMBER>();
}

Status setSerialNumber(std::shared_ptr<Device> device,
                       const std::string& serial_number) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_SERIAL_NUMBER, WRITE)) {
        LOG_ERROR("set serial number is not supported");
        return Status::ERR;
    }
    return device->setOption<OB_LIDAR_OPTION_SERIAL_NUMBER>(serial_number);
}

OB_EXPORT Status initiateConnection(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_INITIATE_DEVICE_CONNECTION,
                                   WRITE)) {
        LOG_ERROR("set initiate connection is not supported");
        return Status::ERR;
    }
    const int32_t magic = 0x12345678;
    return device->setOption<OB_LIDAR_OPTION_INITIATE_DEVICE_CONNECTION>(magic);
}

OB_EXPORT int getSpacialMode(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_SPECIAL_MODE, READ)) {
        LOG_ERROR("get special mode is not supported");
        return -1;
    }
    return device->getOption<OB_LIDAR_OPTION_SPECIAL_MODE>();
}

OB_EXPORT Status setSpacialMode(std::shared_ptr<Device> device,
                                int spacial_mode) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_SPECIAL_MODE, WRITE)) {
        LOG_ERROR("set special mode is not supported");
        return Status::ERR;
    }
    return device->setOption<OB_LIDAR_OPTION_SPECIAL_MODE>(spacial_mode);
}

OB_EXPORT int getWarningInfo(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_WARNING_INFO, READ)) {
        LOG_ERROR("get warning info is not supported");
        return -1;
    }
    return device->getOption<OB_LIDAR_OPTION_WARNING_INFO>();
}

OB_EXPORT int getFilterLevel(std::shared_ptr<Device> device) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_FILTER_LEVEL, READ)) {
        LOG_ERROR("get filter level is not supported");
        return -1;
    }
    return device->getOption<OB_LIDAR_OPTION_FILTER_LEVEL>();
}

OB_EXPORT Status setFilterLevel(std::shared_ptr<Device> device,
                                int filter_level) {
    if (!device->isOptionSupported(OB_LIDAR_OPTION_FILTER_LEVEL, WRITE)) {
        LOG_ERROR("set filter level is not supported");
        return Status::ERR;
    }
    return device->setOption<OB_LIDAR_OPTION_FILTER_LEVEL>(filter_level);
}

std::string asStringLiteral(LidarOption option) {
    switch (option) {
        case OB_LIDAR_OPTION_IP_ADDRESS:
            return "IP_ADDRESS";
        case OB_LIDAR_OPTION_PORT:
            return "PORT";
        case OB_LIDAR_OPTION_MAC_ADDRESS:
            return "MAC_ADDRESS";
        case OB_LIDAR_OPTION_SUBNET_MASK:
            return "SUBNET_MASK";
        case OB_LIDAR_OPTION_SCAN_SPEED:
            return "SCAN_SPEED";
        case OB_LIDAR_OPTION_SCAN_DIRECTION:
            return "SCAN_DIRECTION";
        case OB_LIDAR_OPTION_TRANSFER_PROTOCOL:
            return "TRANSFER_PROTOCOL";
        case OB_LIDAR_OPTION_WORK_MODE:
            return "WORK_MODE";
        case OB_LIDAR_OPTION_INITIATE_DEVICE_CONNECTION:
            return "INITIATE_DEVICE_CONNECTION";
        case OB_LIDAR_OPTION_SERIAL_NUMBER:
            return "SERIAL_NUMBER";
        case OB_LIDAR_OPTION_REBOOT:
            return "REBOOT";
        case OB_LIDAR_OPTION_FACTORY_MODE:
            return "FACTORY_MODE";
        case OB_LIDAR_OPTION_ECHO_MODE:
            return "ECHO_MODE";
        case OB_LIDAR_OPTION_APPLY_CONFIGS:
            return "APPLY_CONFIGS";
        case OB_LIDAR_OPTION_ENABLE_STREAMING:
            return "ENABLE_STREAMING";
        case OB_LIDAR_OPTION_FILTER_LEVEL:
            return "FILTER_LEVEL";
        case OB_LIDAR_OPTION_START_MCU_UPGRADE:
            return "START_MCU_UPGRADE";
        case OB_LIDAR_OPTION_END_MCU_UPGRADE:
            return "END_MCU_UPGRADE";
        case OB_LIDAR_OPTION_PROJECT_ID_VERIFICATION:
            return "PROJECT_ID_VERIFICATION";
        case OB_LIDAR_OPTION_PRODUCT_ID_VERIFICATION:
            return "PRODUCT_ID_VERIFICATION";
        case OB_LIDAR_OPTION_SEND_MD5_VALUE:
            return "SEND_MD5_VALUE";
        case OB_LIDAR_OPTION_VERIFY_MD5_VALUE:
            return "VERIFY_MD5_VALUE";
        case OB_LIDAR_OPTION_TRANSFER_FIRMWARE_UPGRADE_PACKAGE:
            return "TRANSFER_FIRMWARE_UPGRADE_PACKAGE";
        case OB_LIDAR_OPTION_START_FPGA_UPGRADE:
            return "START_FPGA_UPGRADE";
        case OB_LIDAR_OPTION_END_FPGA_UPGRADE:
            return "END_FPGA_UPGRADE";
        case OB_LIDAR_OPTION_TRANSFER_FPGA_UPGRADE_PACKAGE:
            return "TRANSFER_FPGA_UPGRADE_PACKAGE";
        case OB_LIDAR_OPTION_START_MEMS_UPGRADE:
            return "START_MEMS_UPGRADE";
        case OB_LIDAR_OPTION_END_MEMS_UPGRADE:
            return "END_MEMS_UPGRADE";
        case OB_LIDAR_OPTION_MEMS_ID_VERIFICATION:
            return "MEMS_ID_VERIFICATION";
        case OB_LIDAR_OPTION_SEND_MEMS_MD5_VALUE:
            return "SEND_MEMS_MD5_VALUE";
        case OB_LIDAR_OPTION_VERIFY_MEMS_MD5_VALUE:
            return "VERIFY_MEMS_MD5_VALUE";
        case OB_LIDAR_OPTION_TRANSFER_MEMS_UPGRADE_PACKAGE:
            return "TRANSFER_MEMS_UPGRADE_PACKAGE";
        case OB_LIDAR_OPTION_PRODUCT_MODEL:
            return "PRODUCT_MODEL";
        case OB_LIDAR_OPTION_FIRMWARE_VERSION:
            return "FIRMWARE_VERSION";
        case OB_LIDAR_OPTION_FPGA_VERSION:
            return "FPGA_VERSION";
        case OB_LIDAR_OPTION_SPIN_SPEED:
            return "SPIN_SPEED";
        case OB_LIDAR_OPTION_MCU_TEMPERATURE:
            return "MCU_TEMPERATURE";
        case OB_LIDAR_OPTION_FPGA_TEMPERATURE:
            return "FPGA_TEMPERATURE";
        case OB_LIDAR_OPTION_FPGA_VERSION_DATE:
            return "FPGA_VERSION_DATE";
        case OB_LIDAR_OPTION_HIGH_VOLTAGE:
            return "HIGH_VOLTAGE";
        case OB_LIDAR_OPTION_SPECIAL_MODE:
            return "SPECIAL_MODE";
        case OB_LIDAR_OPTION_APD_TEMPERATURE:
            return "APD_TEMPERATURE";
        case OB_LIDAR_OPTION_TX_VOLTAGE:
            return "TX_VOLTAGE";
        default:
            return "UNKNOWN";
    }
}

}  // namespace ob_lidar
