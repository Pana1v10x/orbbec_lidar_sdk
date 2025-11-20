#include "discovery.hpp"
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <iphlpapi.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "iphlpapi.lib")
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#endif

#define MDNS_GROUP "224.0.0.251"
#define MDNS_PORT 5353
#define BUFFER_SIZE 2048

namespace ob_lidar {
char g_buffer[BUFFER_SIZE];

MDNSDiscoveryService::MDNSDiscoveryService() {
    loop_ = uvw::loop::create();
    handle_ = loop_->resource<uvw::udp_handle>();
}

MDNSDiscoveryService::~MDNSDiscoveryService() noexcept { stop(); }

std::vector<unsigned char> constructMDNSQuery() {
    std::vector<unsigned char> query;

    // Transaction ID (0 for mDNS)
    query.push_back(0x00);
    query.push_back(0x00);

    // Flags: standard query, recursion desired
    query.push_back(0x00);
    query.push_back(0x00);

    // Questions, Answer RRs, Authority RRs, Additional RRs
    query.push_back(0x00);  // QDCOUNT
    query.push_back(0x01);  // QDCOUNT = 1
    query.push_back(0x00);  // ANCOUNT
    query.push_back(0x00);
    query.push_back(0x00);  // NSCOUNT
    query.push_back(0x00);
    query.push_back(0x00);  // ARCOUNT
    query.push_back(0x00);

    // Question Section
    std::string service = "_oradar_udp.local";
    size_t pos = 0, prev = 0;
    while ((pos = service.find('.', prev)) != std::string::npos) {
        size_t len = pos - prev;
        query.push_back(static_cast<unsigned char>(len));
        for (size_t i = 0; i < len; ++i) {
            query.push_back(static_cast<unsigned char>(service[prev + i]));
        }
        prev = pos + 1;
    }
    // Last label
    size_t len = service.size() - prev;
    query.push_back(static_cast<unsigned char>(len));
    for (size_t i = 0; i < len; ++i) {
        query.push_back(static_cast<unsigned char>(service[prev + i]));
    }
    // Terminate with zero length
    query.push_back(0x00);

    // Type PTR (12)
    query.push_back(0x00);
    query.push_back(0x0C);

    // Class IN (1) and cache flush (bit 15)
    query.push_back(0x80);  // 0x8001: Cache flush
    query.push_back(0x01);

    return query;
}

#ifdef _WIN32
std::vector<std::string> getAllIPv4Addresses() {
    std::vector<std::string> ipv4Addresses;

    auto wsaData = std::shared_ptr<WSADATA>(new WSADATA(), [](WSADATA* wsa) {
        WSACleanup();
        delete wsa;
    });

    if (WSAStartup(MAKEWORD(2, 2), wsaData.get()) != 0) {
        std::cerr << "WSAStartup failed." << std::endl;
        return ipv4Addresses;
    }

    ULONG bufferSize = 0;
    GetAdaptersAddresses(AF_UNSPEC, 0, NULL, NULL, &bufferSize);

    auto buffer = std::shared_ptr<char[]>(new char[bufferSize]);
    auto adapterAddresses =
        reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buffer.get());

    if (GetAdaptersAddresses(AF_UNSPEC, GAA_FLAG_INCLUDE_PREFIX, NULL,
                             adapterAddresses, &bufferSize) == NO_ERROR) {
        for (auto adapter = adapterAddresses; adapter != nullptr;
             adapter = adapter->Next) {
            for (auto unicast = adapter->FirstUnicastAddress;
                 unicast != nullptr; unicast = unicast->Next) {
                if (unicast->Address.lpSockaddr->sa_family == AF_INET) {
                    auto ipv4 = reinterpret_cast<sockaddr_in*>(
                        unicast->Address.lpSockaddr);

                    auto ipStr =
                        std::shared_ptr<char[]>(new char[INET_ADDRSTRLEN]);
                    if (inet_ntop(AF_INET, &(ipv4->sin_addr), ipStr.get(),
                                  INET_ADDRSTRLEN)) {
                        ipv4Addresses.emplace_back(ipStr.get());
                    }
                }
            }
        }
    } else {
        std::cerr << "GetAdaptersAddresses failed." << std::endl;
    }

    return ipv4Addresses;
}

#else
// Helper function to enumerate all IPv4 addresses on Linux
std::vector<std::string> getAllIPv4AddressesLinux() {
    std::vector<std::string> ipv4_addresses;

    struct ifaddrs* ifaddr;
    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return ipv4_addresses;
    }

    for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;

        if (ifa->ifa_addr->sa_family == AF_INET) {  // IPv4
            char addrStr[INET_ADDRSTRLEN];
            struct sockaddr_in* sa_in =
                reinterpret_cast<struct sockaddr_in*>(ifa->ifa_addr);
            inet_ntop(AF_INET, &(sa_in->sin_addr), addrStr, INET_ADDRSTRLEN);
            ipv4_addresses.emplace_back(addrStr);
        }
    }

    freeifaddrs(ifaddr);
    return ipv4_addresses;
}
#endif

void MDNSDiscoveryService::start(const onNewDeviceCallback& callback) {
    on_new_device_callback_ = callback;

#ifdef _WIN32
    // Retrieve all IPv4 addresses
    auto ipv4_addresses = getAllIPv4Addresses();
    if (ipv4_addresses.empty()) {
        std::cerr << "No IPv4 addresses found on Windows." << std::endl;
        return;
    }

#else
    // Retrieve all IPv4 addresses on Linux
    auto ipv4_addresses = getAllIPv4AddressesLinux();
    if (ipv4_addresses.empty()) {
        std::cerr << "No IPv4 addresses found on Linux." << std::endl;
        return;
    }
#endif

    // Bind to all interfaces
    handle_->bind("0.0.0.0", MDNS_PORT, uvw::details::uvw_udp_flags::REUSEADDR);
    handle_->multicast_ttl(255);  // Set TTL to 255 for multicast packets
    // Join multicast group on each interface
    for (const auto& interface_ip : ipv4_addresses) {
        bool joined = handle_->multicast_membership(
            MDNS_GROUP, interface_ip, uvw::udp_handle::membership::JOIN_GROUP);
        if (joined) {
            std::cout << "Joined multicast group on interface: " << interface_ip
                      << std::endl;
        } else {
            std::cerr << "Failed to join multicast group on interface: "
                      << interface_ip << std::endl;
        }
    }

    handle_->on<uvw::error_event>(
        [](const uvw::error_event& err, uvw::udp_handle&) {
            std::cerr << "Error: " << err.name() << " - " << err.what()
                      << std::endl;
        });

    handle_->on<uvw::udp_data_event>(
        [this](const uvw::udp_data_event& event, uvw::udp_handle&) {
            auto src_ip = event.sender.ip;

            memset(g_buffer, 0, BUFFER_SIZE);
            memcpy(g_buffer, event.data.get(), event.length);
            parseMDNSResponse(reinterpret_cast<unsigned char*>(g_buffer),
                              event.length, src_ip);
        });

    handle_->recv();

    // Send mDNS query

    send_query_ = true;
    query_thread_ = std::make_shared<std::thread>([this] {
        while (send_query_) {
            auto query = constructMDNSQuery();
            uvw::socket_address addr{MDNS_GROUP, MDNS_PORT};
            handle_->send(addr, reinterpret_cast<char*>(query.data()),
                          query.size());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    loop_thread_ = std::make_shared<std::thread>([this] { loop_->run(); });
}

void MDNSDiscoveryService::stop() {
    if (loop_thread_ && loop_thread_->joinable()) {
        loop_->stop();
        loop_->walk([](auto&& handle) { handle.close(); });
        loop_->close();
        loop_thread_->detach();  // Detach the thread if it is still running
    }
    send_query_ = false;
    if (query_thread_ && query_thread_->joinable()) {
        query_thread_->join();
    }
    query_thread_.reset();
}

const unsigned char* MDNSDiscoveryService::parseDomainNames(
    const unsigned char* buffer, const unsigned char* ptr, std::string& name) {
    const unsigned char* end = buffer + BUFFER_SIZE;
    while (ptr < end) {
        int len = *ptr++;
        if (len == 0) {
            return ptr;
        }
        // Compressed pointer to another location in the packet
        if ((len & 0xC0) == 0xC0) {
            int offset = ((len & 0x3F) << 8) | *ptr++;
            parseDomainNames(buffer, buffer + offset, name);
            return ptr;
        }
        if (ptr + len > end) break;
        name.append(reinterpret_cast<const char*>(ptr), len);
        name.append(".");
        ptr += len;
    }
    return ptr;
}

void MDNSDiscoveryService::parseMDNSResponse(const unsigned char* buffer,
                                             int length,
                                             const std::string& src_ip) {
    if (length < 12) {
        std::cerr << "Received packet is too short." << std::endl;
        return;
    }

    const unsigned char* ptr = buffer + 12;  // Skip the DNS header
    std::string name;

    // Parse the Question section (usually skipped since it's often empty)
    int question_count = ntohs(*(uint16_t*)(buffer + 4));
    for (int i = 0; i < question_count; ++i) {
        ptr = parseDomainNames(buffer, ptr, name) + 4;  // Skip Type and Class
    }

    // Parse the Answer section
    int answer_count = ntohs(*(uint16_t*)(buffer + 6));
    for (int i = 0; i < answer_count; ++i) {
        name.clear();
        ptr = parseDomainNames(buffer, ptr, name);  // Parse the record name
        if (ptr + 10 > buffer + length) {
            std::cerr << "Packet malformed while reading answer." << std::endl;
            return;
        }

        uint16_t type = ntohs(*(uint16_t*)ptr);  // Record type
        ptr += 2;
        uint16_t cls = ntohs(*(uint16_t*)ptr);  // Record class
        ptr += 2;
        uint32_t ttl = ntohl(*(uint32_t*)ptr);  // TTL
        ptr += 4;
        uint16_t data_length = ntohs(*(uint16_t*)ptr);  // Data length
        ptr += 2;

        if (ptr + data_length > buffer + length) {
            std::cerr << "Packet malformed: data length exceeds buffer."
                      << std::endl;
            return;
        }

        // Only process SRV records where the service name contains
        // "_oradar_udp" SRV record type is 33
        if (type == 33 && name.find("_oradar_udp") != std::string::npos) {
            if (data_length < 6) {
                std::cerr << "SRV record data too short." << std::endl;
                return;
            }
            uint16_t priority = ntohs(*(uint16_t*)ptr);
            uint16_t weight = ntohs(*(uint16_t*)(ptr + 2));
            uint16_t port = ntohs(*(uint16_t*)(ptr + 4));
            std::string target;
            parseDomainNames(buffer, ptr + 6, target);
            if (on_new_device_callback_) {
                on_new_device_callback_(src_ip, port);
            }
        }
        ptr += data_length;
    }
}
}  // namespace ob_lidar
