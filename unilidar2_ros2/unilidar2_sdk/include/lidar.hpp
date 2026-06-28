#pragma once

#include <string.h>
#include <stdexcept>
#include <memory>
#include <thread>
#include <queue>
#include <mutex>
#include <atomic>
#include <chrono>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "messages.hpp"
#include "decoder.hpp"
#include "out_buffer.hpp"

#define SET_FRAME_HEADER(t, type)                  \
    packet.header.header[0] = FRAME_HEADER_BYTE_0; \
    packet.header.header[1] = FRAME_HEADER_BYTE_1; \
    packet.header.header[2] = FRAME_HEADER_BYTE_2; \
    packet.header.header[3] = FRAME_HEADER_BYTE_3; \
    packet.header.packet_type = type;              \
    packet.header.packet_size = sizeof(t);

#define SET_FRAME_TAIL                       \
    packet.tail.tail[0] = FRAME_TAIL_BYTE_0; \
    packet.tail.tail[1] = FRAME_TAIL_BYTE_1;
#define CRC32(t) packet.tail.crc32 = crc32(crc32(0L, Z_NULL, 0), reinterpret_cast<const Bytef *>(&packet.data), sizeof(t));

#define SEND_PACKET(t) sendto(sock_fd_, reinterpret_cast<const char *>(&packet), sizeof(t), 0, (const struct sockaddr *)&remote_addr_, sizeof(remote_addr_));

class Lidar
{
private:
    TimeStamp last_imu{};

    int sock_fd_;
    struct sockaddr_in local_addr_;
    struct sockaddr_in remote_addr_;

    uint8_t buffer_[4096];
    size_t buffer_used_ = 0;

    std::atomic<bool> running_;
    std::atomic<bool> expect_cmd_ack_;
    std::atomic<bool> wait_for_cmd_ack_;

    OutBuffer out_buffer_;
    std::mutex mutex_;
    std::unique_ptr<std::thread> rx_thread_;

    void buffer_packet(const DecodeRes &res);

    void rx_worker();

public:
    Lidar() = delete;
    Lidar(const char *local_ip, int local_port, const char *remote_ip, int remote_port);
    ~Lidar();

    bool wait_for_ack(int64_t timeout_ms);

    void send_cmd(uint32_t type, uint32_t value);
    void request_version();
    void sync_time(uint32_t time_sec, uint32_t time_nsec);
    void set_work_mode(bool wide_fov);

    L2Cloud get_cloud();
    L2Imu get_imu();
};