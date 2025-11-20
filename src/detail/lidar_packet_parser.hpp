#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>

#include "utils.hpp"

#ifdef _WIN32
#include <winsock2.h>
#elif __linux__
#include <arpa/inet.h>
#endif

#ifdef _WIN32
inline uint64_t be64toh(uint64_t value) noexcept {
    return ((value & 0xFF00000000000000) >> 56) |
           ((value & 0x00FF000000000000) >> 40) |
           ((value & 0x0000FF0000000000) >> 24) |
           ((value & 0x000000FF00000000) >> 8) |
           ((value & 0x00000000FF000000) << 8) |
           ((value & 0x0000000000FF0000) << 24) |
           ((value & 0x000000000000FF00) << 40) |
           ((value & 0x00000000000000FF) << 56);
}
#endif

namespace ob_lidar {
inline uint16_t ntohs_wrapper(uint16_t netshort) {
    return ntohs(netshort);  // original macro function
}

inline uint32_t ntohl_wrapper(uint32_t netlong) { return ntohl(netlong); }

#pragma pack(push, 1)

struct CommandResponseHeader {
    uint16_t header;           // 2 bytes
    uint8_t protocol_version;  // 1 byte
    uint16_t frame_length;     // 2 bytes
    uint16_t command_id;       // 2 bytes
    uint16_t status;           // 2 bytes

    uint16_t getHeader() const noexcept { return ntohs_wrapper(header); }
    uint8_t getProtocolVersion() const noexcept { return protocol_version; }
    uint16_t getFrameLength() const noexcept {
        return ntohs_wrapper(frame_length);
    }
    uint16_t getCommandId() const noexcept { return ntohs_wrapper(command_id); }
    uint16_t getStatus() const noexcept { return ntohs_wrapper(status); }
    size_t getPayloadSize() const noexcept { return getFrameLength(); }
};

struct MultiLineDataPacketHeader {
    uint32_t packet_header1;      // 4 bytes
    uint16_t packet_header2;      // 2 bytes
    uint16_t packet_length;       // 2 bytes
    uint8_t lidar_type;           // 1 byte
    uint8_t scan_frequency_type;  // 1 byte
    uint8_t block_count;          // 1 byte
    uint16_t frame_index;         // 2 bytes
    uint8_t block_data_num;       // 1 byte
    uint8_t data_type;            // 1 byte
    uint64_t timestamp;           // 8 bytes
    uint8_t sync_mode;            // 1 byte
    uint32_t warning_status;      // 4 bytes
    uint8_t echo_mode;            // 1 byte
    uint16_t horizontal_rpm;      // 2 bytes
    uint16_t vertical_frequency;  // 2 bytes
    uint16_t apd_temperature;     // 2 bytes
    uint8_t reserved[5];          // 5 bytes

    uint32_t getPacketHeader1() const noexcept {
        return ntohl_wrapper(packet_header1);
    }
    uint16_t getPacketHeader2() const noexcept {
        return ntohs_wrapper(packet_header2);
    }
    uint16_t getPacketLength() const noexcept {
        return ntohs_wrapper(packet_length);
    }
    uint8_t getLidarType() const noexcept { return lidar_type; }
    uint8_t getScanFrequencyType() const noexcept {
        return scan_frequency_type;
    }
    uint8_t getBlockCount() const noexcept { return block_count; }
    uint16_t getFrameIndex() const noexcept {
        return ntohs_wrapper(frame_index);
    }
    uint8_t getBlockDataNum() const noexcept { return block_data_num; }
    uint8_t getDataType() const noexcept { return data_type; }
    uint64_t getTimestamp() const noexcept { return be64toh(timestamp); }
    uint8_t getSyncMode() const noexcept { return sync_mode; }
    uint32_t getWarningStatus() const noexcept {
        return ntohl_wrapper(warning_status);
    }
    uint8_t getEchoMode() const noexcept { return echo_mode; }
    uint16_t getHorizontalRpm() const noexcept {
        return ntohs_wrapper(horizontal_rpm);
    }
    uint16_t getVerticalFrequency() const noexcept {
        return ntohs_wrapper(vertical_frequency);
    }
    uint16_t getApdTemperature() const noexcept {
        return ntohs_wrapper(apd_temperature);
    }

    size_t getPayloadSize() const noexcept {
        constexpr size_t min_size =
            sizeof(MultiLineDataPacketHeader) + sizeof(uint32_t);
        return getPacketLength() < min_size ? 0 : getPacketLength() - min_size;
    }
};

struct SingleLineDataPacketHeader {
    uint32_t packet_header1;      // 4 bytes
    uint16_t packet_header2;      // 2 bytes
    uint16_t packet_length;       // 2 bytes
    uint16_t start_angle;         // 2 bytes
    uint16_t end_angle;           // 2 bytes
    uint16_t angle_resolution;    // 2 bytes
    uint8_t data_type;            // 1 byte
    uint8_t block_index;          // 1 byte
    uint16_t frame_index;         // 2 bytes
    uint32_t timestamp;           // 4 bytes
    uint8_t sync_mode;            // 1 byte
    uint8_t special_mode;         // 1 byte
    uint32_t warning_info;        // 4 bytes
    uint16_t contaminated_angle;  // 2 bytes
    uint8_t contaminated_level;   // 1 byte
    uint16_t apd_temperature;     // 2 bytes
    uint16_t spin_speed;          // 2 bytes
    uint8_t reserved[5];          // 5 bytes

    inline uint32_t getPacketHeader1() const noexcept {
        return ntohl_wrapper(packet_header1);
    }
    inline uint16_t getPacketHeader2() const noexcept {
        return ntohs_wrapper(packet_header2);
    }
    inline uint16_t getPacketLength() const noexcept {
        return ntohs_wrapper(packet_length);
    }
    inline double getStartAngle() const noexcept {
        return ntohs_wrapper(start_angle) * 0.01;
    }
    inline double getEndAngle() const noexcept {
        return ntohs_wrapper(end_angle) * 0.01;
    }
    inline double getAngleResolution() const noexcept {
        return ntohs_wrapper(angle_resolution) * 0.001;
    }
    inline uint8_t getDataType() const noexcept { return data_type; }
    inline uint8_t getBlockIndex() const noexcept { return block_index; }
    inline uint16_t getFrameIndex() const noexcept {
        return ntohs_wrapper(frame_index);
    }
    inline uint32_t getTimestamp() const noexcept {
        return ntohl_wrapper(timestamp);
    }
    inline uint8_t getSyncMode() const noexcept { return sync_mode; }
    inline uint8_t getSpecialMode() const noexcept { return special_mode; }
    inline uint32_t getWarningInfo() const noexcept {
        return ntohl_wrapper(warning_info);
    }
    inline uint16_t getContaminatedAngle() const noexcept {
        return ntohs_wrapper(contaminated_angle);
    }
    inline uint8_t getContaminatedLevel() const noexcept {
        return contaminated_level;
    }
    inline uint16_t getApdTemperature() const noexcept {
        return ntohs_wrapper(apd_temperature);
    }
    inline uint16_t getSpinSpeed() const noexcept {
        return ntohs_wrapper(spin_speed);
    }

    inline size_t getPayloadSize() const noexcept {
        const size_t min_size =
            sizeof(SingleLineDataPacketHeader) + sizeof(uint32_t);
        return getPacketLength() < min_size ? 0 : getPacketLength() - min_size;
    }
};

#pragma pack(pop)

class LidarPacketParserError : public std::runtime_error {
   public:
    explicit LidarPacketParserError(const std::string& msg)
        : std::runtime_error(msg) {}
};

// Header trait specializations
template <LidarPacketType type>
struct LidarPacketHeaderTrait {};

template <>
struct LidarPacketHeaderTrait<LidarPacketType::COMMAND> {
    using type = CommandResponseHeader;
};

template <>
struct LidarPacketHeaderTrait<LidarPacketType::SINGLE_LINE> {
    using type = SingleLineDataPacketHeader;
};

template <>
struct LidarPacketHeaderTrait<LidarPacketType::MULTI_LINE> {
    using type = MultiLineDataPacketHeader;
};

template <LidarPacketType type>
class LidarPacket {
   public:
    using HeaderT = typename LidarPacketHeaderTrait<type>::type;

    LidarPacket(const uint8_t* data, size_t size) {
        if (!data || size < sizeof(HeaderT)) {
            throw LidarPacketParserError("Invalid packet data or size");
        }

        data_ = data;
        data_size_ = size;
        std::memcpy(&header_, data, sizeof(HeaderT));

        if (type == LidarPacketType::COMMAND) {
            validateCommandPacket();
        }
    }

    LidarPacket(const LidarPacket&) = delete;
    LidarPacket& operator=(const LidarPacket&) = delete;
    LidarPacket(LidarPacket&&) noexcept = default;
    LidarPacket& operator=(LidarPacket&&) noexcept = default;

    inline const HeaderT* header() const noexcept { return &header_; }
    inline const uint8_t* payload() const noexcept {
        return data_ + sizeof(HeaderT);
    }
    inline size_t payloadSize() const noexcept {
        return header_.getPayloadSize();
    }

   private:
    inline void validateCommandPacket() const {
        if (data_size_ < 2) {
            throw LidarPacketParserError("Command packet too small for CRC");
        }

        uint8_t crc = data_[data_size_ - 1];
        uint8_t expected_crc = calcCrc8(data_, data_size_ - 1);

        if (crc != expected_crc) {
            throw LidarPacketParserError("Invalid command response packet CRC");
        }
    }

    HeaderT header_{};
    const uint8_t* data_{nullptr};
    size_t data_size_{0};
};

class LidarPacketParser {
   public:
    template <LidarPacketType T>
    static std::shared_ptr<LidarPacket<T>> parse(const uint8_t* data,
                                                 size_t size) {
        if (!data) {
            throw LidarPacketParserError("Null data pointer");
        }

        using HeaderType = typename LidarPacketHeaderTrait<T>::type;
        if (size < sizeof(HeaderType)) {
            throw LidarPacketParserError("Packet size too small for header");
        }

        return std::make_shared<LidarPacket<T>>(data, size);
    }
};

using CommandPacketPtr = std::shared_ptr<LidarPacket<LidarPacketType::COMMAND>>;
using SingleLinePacketPtr =
    std::shared_ptr<LidarPacket<LidarPacketType::SINGLE_LINE>>;
using MultiLinePacketPtr =
    std::shared_ptr<LidarPacket<LidarPacketType::MULTI_LINE>>;

}  // namespace ob_lidar
