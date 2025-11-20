#pragma once
#include <atomic>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <uvw.hpp>

namespace ob_lidar {

using onNewDeviceCallback =
    std::function<void(const std::string& ip, const uint16_t& port)>;

class MDNSDiscoveryService {
   public:
    MDNSDiscoveryService();

    ~MDNSDiscoveryService() noexcept;

    void start(const onNewDeviceCallback& callback);

    void stop() ;

    // Function to parse domain names from compressed mDNS packets
    static const unsigned char* parseDomainNames(const unsigned char* buffer,
                                                 const unsigned char* ptr,
                                                 std::string& name);

    void parseMDNSResponse(const unsigned char* buffer, int length,
                           const std::string& src_ip);

   private:
    std::shared_ptr<uvw::loop> loop_ = nullptr;
    std::shared_ptr<uvw::udp_handle> handle_ = nullptr;
    std::shared_ptr<std::thread> loop_thread_ = nullptr;
    std::shared_ptr<std::thread> query_thread_ = nullptr;
    std::atomic_bool send_query_{false};
    onNewDeviceCallback on_new_device_callback_;
};
}  // namespace ob_lidar
