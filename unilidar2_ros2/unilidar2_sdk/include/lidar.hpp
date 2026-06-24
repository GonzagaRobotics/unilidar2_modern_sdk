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

#define SEND_PACKET(t) sendto(sock_fd_, &packet, sizeof(t), 0, (const struct sockaddr *)&remote_addr_, sizeof(remote_addr_));

#define CRC32(t) packet.tail.crc32 = crc32(crc32(0L, Z_NULL, 0), reinterpret_cast<const Bytef *>(&packet.data), sizeof(t));

class Lidar
{
private:
    int sock_fd_;
    struct sockaddr_in local_addr_;
    struct sockaddr_in remote_addr_;
    uint8_t buffer_[2048];

    std::atomic<bool> running_;
    std::atomic<bool> wait_for_cmd_ack_;

    uint32_t time_sec_;
    uint32_t time_nsec_;

    OutBuffer out_buffer_;
    std::mutex mutex_;
    std::unique_ptr<std::thread> rx_thread_;

    void buffer_packet(const DecodeRes &res);

    void rx_worker();

public:
    Lidar() = delete;
    Lidar(const char *local_ip, int local_port, const char *remote_ip, int remote_port);
    ~Lidar();

    void sync_time(uint32_t time_sec, uint32_t time_nsec);
    void set_work_mode(bool wide_fov, bool cloud_2d, bool disable_imu, bool use_serial, bool start_standby);

    L2Cloud get_cloud();
    L2Imu get_imu();
};