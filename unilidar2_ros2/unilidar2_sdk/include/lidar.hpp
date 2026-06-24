#pragma once

#include <string.h>
#include <stdexcept>
#include <thread>
#include <queue>
#include <mutex>
#include <atomic>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "messages.hpp"
#include "decoder.hpp"

#define VAL_PACKET_TYPE 1

class Lidar
{
private:
    struct BufferedPacket
    {
        uint32_t packet_type;
        uint8_t *data;
    };

    int sock_fd_;
    struct sockaddr_in local_addr_;
    struct sockaddr_in remote_addr_;
    uint8_t buffer_[2048];

    std::atomic<bool> running_;

    std::queue<BufferedPacket> packet_buffer_;
    std::mutex mutex_;
    std::thread rx_thread_;

    void rx_worker();

public:
    Lidar() = delete;
    Lidar(const char *local_ip, int local_port, const char *remote_ip, int remote_port);
    ~Lidar();

    int has_data();
    void ignore_data();

    PointData get_point_data();
    ImuData get_imu_data();
};