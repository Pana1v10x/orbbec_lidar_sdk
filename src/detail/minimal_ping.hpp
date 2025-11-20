#pragma once

#include <chrono>
#include <iostream>
#include <string>

#ifdef __linux__
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/ip_icmp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#endif

class MinimalPing {
   public:
    MinimalPing() = default;

    ~MinimalPing() = default;

    static bool ping(const std::string &address, int timeout_ms = 1000,
                     int retries = 3) {
#ifdef _WIN32
        return pingWindows(address, timeout_ms, retries);
#else
        return pingLinux(address, timeout_ms, retries);
#endif
    }

   private:
#ifdef _WIN32
    static bool pingWindows(const std::string &address, int timeout_ms,
                            int retries) {
        (void)timeout_ms;
        (void)retries;
        (void)address;
        return true;
    }
#else
    static bool pingLinux(const std::string &address, int timeout_ms,
                          int retries) {
        bool success = false;
        for (int i = 0; i < retries && !success; i++) {
            int sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_ICMP);
            if (sockfd < 0) {
                std::cerr << "Socket creation failed! Error: "
                          << strerror(errno) << std::endl;
                return false;
            }

            // Set both send and receive timeouts
            struct timeval timeout;
            timeout.tv_sec = timeout_ms / 1000;
            timeout.tv_usec = (timeout_ms % 1000) * 1000;

            if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                           sizeof(timeout)) < 0 ||
                setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &timeout,
                           sizeof(timeout)) < 0) {
                std::cerr << "Failed to set socket options!" << std::endl;
                close(sockfd);
                return false;
            }

            sockaddr_in dest_addr;
            memset(&dest_addr, 0, sizeof(dest_addr));
            dest_addr.sin_family = AF_INET;

            if (inet_pton(AF_INET, address.c_str(), &dest_addr.sin_addr) != 1) {
                std::cerr << "Invalid IP address." << std::endl;
                close(sockfd);
                return false;
            }

            char packet[8];
            memset(packet, 0, sizeof(packet));
            struct icmphdr *icmp_hdr =
                reinterpret_cast<struct icmphdr *>(packet);
            icmp_hdr->type = ICMP_ECHO;
            icmp_hdr->code = 0;
            icmp_hdr->un.echo.id = getpid() & 0xFFFF;
            icmp_hdr->un.echo.sequence = i + 1;
            icmp_hdr->checksum = calculateChecksum(packet, sizeof(packet));

            auto start_time = std::chrono::high_resolution_clock::now();

            if (sendto(sockfd, packet, sizeof(packet), 0,
                       reinterpret_cast<sockaddr *>(&dest_addr),
                       sizeof(dest_addr)) <= 0) {
                std::cerr << "Packet sending failed! Error: " << strerror(errno)
                          << std::endl;
                close(sockfd);
                if (i < retries - 1) {
                    usleep(100000);  // Wait 100ms between retries
                    continue;
                }
                return false;
            }

            char buffer[1024];
            sockaddr_in recv_addr;
            socklen_t addr_len = sizeof(recv_addr);

            int recv_result =
                recvfrom(sockfd, buffer, sizeof(buffer), 0,
                         reinterpret_cast<sockaddr *>(&recv_addr), &addr_len);

            if (recv_result > 0) {
                auto end_time = std::chrono::high_resolution_clock::now();
                auto rtt =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        end_time - start_time)
                        .count();
                success = true;
            } else {
                std::cerr << "Receive failed on attempt " << (i + 1)
                          << ". Error: " << strerror(errno) << std::endl;
            }

            close(sockfd);

            if (!success && i < retries - 1) {
                usleep(100000);  // Wait 100ms between retries
            }
        }

        return success;
    }

    static uint16_t calculateChecksum(void *b, int len) {
        uint16_t *buf = static_cast<uint16_t *>(b);
        uint32_t sum = 0;
        uint16_t result;

        for (sum = 0; len > 1; len -= 2) sum += *buf++;
        if (len == 1) sum += *(uint8_t *)buf;
        sum = (sum >> 16) + (sum & 0xFFFF);
        sum += (sum >> 16);
        result = ~sum;
        return result;
    }
#endif
};
