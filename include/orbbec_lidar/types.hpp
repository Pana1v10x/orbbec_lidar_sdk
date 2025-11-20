#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>
namespace ob_lidar {

enum class Status : int {
    OK = 0,
    ERR = 1,
    OUT_OF_RANGE = 2,
    TIMEOUT = 3,
    INVALID = 4,
    NOT_SUPPORTED = 5,
    INTERNAL_ERROR = 6,
};

struct LidarPoint {
    int32_t x;            /**< X axis, Unit: 2mm */
    int32_t y;            /**< Y axis, Unit: 2mm */
    int32_t z;            /**< Z axis, Unit: 2mm */
    uint8_t reflectivity; /**< Reflectivity */

    /**
     * Tag:
     * bit6-7: Reserved
     * bit4-5: Detection Point Attribute -
     *     0: High Confidence (Normal Point)
     *     1: Medium Confidence
     *     2: Low Confidence
     *     3: Reserved
     * bit2-3: Detection Point Attribute - Rain, Fog, Dust, and Other Tiny
     * Particles 0: High Confidence (Normal Point) 1: Medium Confidence 2: Low
     * Confidence 3: Reserved bit0-1: Detection Point Attribute - Drag Point
     * Between Foreground and Background Objects 0: High Confidence (Normal
     * Point) 1: Medium Confidence 2: Low Confidence 3: Reserved
     */
    uint8_t tag;
};

struct LidarSpherePoint {
    uint32_t depth; /**< Distance, Unit: 2mm */
    uint16_t theta; /**< Zenith angle, Unit: 0.01 degree */
    uint16_t phi;   /**< Azimuth angle, Unit: 0.01 degree */
    uint8_t reflectivity;

    /**
     * @copydoc LidarPoint::tag
     */
    uint8_t tag;
};

/**
 * @brief Represents a point cloud captured by a lidar sensor.
 */
struct LidarPointCloud {
    uint64_t timestamp; /**< The timestamp of the point cloud. */
    std::vector<LidarPoint>
        points; /**< The collection of lidar points in the point cloud. */
};

/**
 * @brief Represents a single lidar scan.
 */
struct LidarScan {
    uint64_t timestamp;           /**< The timestamp of the scan. */
    double start_angle;           /**< The starting angle of the scan. */
    double end_angle;             /**< The ending angle of the scan. */
    double angle_resolution;      /**< The angular resolution of the scan. */
    double contaminated_angle;    /**< The angle range that is contaminated. */
    uint8_t contaminated_level;   /**< The contamination level of the scan. */
    std::vector<uint16_t> ranges; /**< The range measurements of the scan. */
    std::vector<uint16_t>
        intensities; /**< The intensity measurements of the scan. */
};

/**
 * @brief Protocol type for the Lidar device.
 */
enum class LidarProtocolType : int {
    UDP,
    TCP,
    COUNT,
    UNKNOWN = 0xFF,
};

/**
 * @brief Work mode for the Lidar device.
 */
enum class LidarWorkMode : int {
    MEASURE = 0,  // Measure
    STANDBY = 1,  // Standby
    SLEEP = 3,    // Sleep
    UNKNOWN = 0xFF,
};

/**
 * @brief Frame type for the Lidar device.
 */
enum class LidarFrameType : int {
    IMU,
    POINT_CLOUD,
    SPHERE_POINT_CLOUD,
    SCAN,
    LOG,
    COUNT,
    UNKNOWN = 0xFF,
};

/**
 * @brief Stream type for the Lidar device.
 */
enum class LidarStreamType : int {
    IMU,
    POINT_CLOUD,
    SPHERE_POINT_CLOUD,
    SCAN,
    LOG,
    COUNT,
    UNKNOWN = 0xFF,
};

/**
 * @brief Channel type for the Lidar device.
 */
enum class LidarChannelType : int {
    COMMAND,             // command channel
    POINT_CLOUD,         // point cloud channel
    SPHERE_POINT_CLOUD,  // sphere point cloud channel
    IMU,                 // imu channel
    LOG,                 // log channel
    COUNT,               // channel count
    UNKNOWN,
};

/**
 * @brief Status flags for the Lidar device.
 */
enum class LidarStatusFlags : int {
    // 0-1 bits: MCU temperature
    // 00: Temperature normal
    // 01: Temperature high
    // 10: Temperature low
    // 11: Reserved
    MCU_TEMPERATURE_NORMAL = 0x00000000,
    MCU_TEMPERATURE_HIGH = 0x00000001,
    MCU_TEMPERATURE_LOW = 0x00000002,
    MCU_TEMPERATURE_RESERVED = 0x00000003,

    // 2-3 bits: FPGA temperature
    // 00: Temperature normal
    // 01: Temperature high
    // 10: Temperature low
    FPGA_TEMPERATURE_NORMAL = 0x00000000 << 2,
    FPGA_TEMPERATURE_HIGH = 0x00000001 << 2,
    FPGA_TEMPERATURE_LOW = 0x00000002 << 2,

    // 3-4 bits: APD bias voltage
    // 00: Voltage normal
    // 01: Voltage high
    // 10: Voltage low
    APD_BIAS_VOLTAGE_NORMAL = 0x00000000 << 3,
    APD_BIAS_VOLTAGE_HIGH = 0x00000001 << 3,
    APD_BIAS_VOLTAGE_LOW = 0x00000002 << 3,

    // 5-6 bits: Motor status
    // 00: Normal
    // 01: Motor speed abnormal
    // 10: Motor blocked
    MOTOR_STATUS_NORMAL = 0x00000000 << 5,
    MOTOR_STATUS_ABNORMAL = 0x00000001 << 5,
    MOTOR_STATUS_BLOCKED = 0x00000002 << 5,

    // 7-8 bits: Dirty detection
    // 00: No dirt
    // 01: Cover dirty
    DIRTY_DETECTION_NONE = 0x00000000 << 7,
    DIRTY_DETECTION_COVER = 0x00000001 << 7,

    // 9th bit: Network detection
    // 0: Normal
    // 1: Network disconnected
    NETWORK_DETECTION_NORMAL = 0x00000000 << 9,
    NETWORK_DETECTION_DISCONNECTED = 0x00000001 << 9,

    // 10-31 bits: Reserved
    RESERVED = 0x00000000 << 10,
};

/**
 * @brief Echo mode for the Lidar device.
 */
enum class LidarEchoMode : int {
    FIRST = 0,  // first echo
    LAST = 1,   // last echo
    UNKNOWN = 0xFF,
};

struct DeviceInfo {
    std::string device_name;
    std::string lidar_type;
    std::string model;
    std::string serial_number;
    std::string firmware_version;
    std::string fpga_version;
    std::string ip;
    uint16_t port;
};

/**
 * @brief Product model type for the Lidar device.
 */
enum class LidarType : uint8_t {
    SINGLE_LINE,
    MULTI_LINE,
    UNKNOWN,
};

/**
 * @brief Log level for the Lidar device.
 */
enum class LogLevel : int {
    INFO,
    WARNING,
    ERR,
    TRACE,
    DEBUG,
    CRITICAL,
};

struct LogMessage {
    LogLevel level;
    std::string message;
};

using logMessageCallback = std::function<void(const LogMessage&)>;

enum class LidarPacketType : uint8_t {
    COMMAND = 0x01,
    SINGLE_LINE = 0x02,
    MULTI_LINE = 0x03,
};

}  // namespace ob_lidar
