#include <zlib.h>
#include "lidar.hpp"

void Lidar::buffer_packet(const DecodeRes &res)
{
    switch (res.header.packet_type)
    {
    case POINT_DATA_PACKET_TYPE:
        out_buffer_.add_points(*reinterpret_cast<PointData *>(res.data));
        break;
    case IMU_DATA_PACKET_TYPE:
        out_buffer_.add_imu(*reinterpret_cast<ImuData *>(res.data));
        break;
    }
}

void Lidar::rx_worker()
{
    while (running_)
    {
        ssize_t n = recvfrom(sock_fd_, buffer_, sizeof(buffer_), 0, nullptr, nullptr);

        if (n < 0)
        {
            std::cerr << "recvfrom failed: " << strerror(errno) << std::endl;
            continue;
        }

        uint8_t *curr_start = buffer_;
        ssize_t curr_size = n;

        while (curr_size > 0)
        {
            try
            {
                DecodeRes res = decode_packet(curr_start, curr_size);
                curr_start += res.bytes_parsed;
                curr_size -= res.bytes_parsed;

                std::lock_guard<std::mutex> lock(mutex_);

                // Ignore non-ack packets while waiting for an ack
                if (wait_for_cmd_ack_)
                {
                    // TODO: Check that we have gotten the right ack packet for the command we sent
                    if (res.header.packet_type != ACK_DATA_PACKET_TYPE)
                    {
                        // Data needs to be freed since it won't be added to the packet buffer
                        delete[] res.data;
                        continue;
                    }

                    wait_for_cmd_ack_ = false;
                }

                buffer_packet(res);
                delete[] res.data; // Free the memory allocated for the packet data
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error decoding packet: " << e.what() << std::endl;
            }
        }
    }
}

Lidar::Lidar(const char *local_ip, int local_port, const char *remote_ip, int remote_port)
{
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

    if (sock_fd_ < 0)
    {
        throw std::runtime_error("socket creation failed");
    }

    memset(&local_addr_, 0, sizeof(local_addr_));
    local_addr_.sin_family = AF_INET;
    local_addr_.sin_addr.s_addr = inet_addr(local_ip);
    local_addr_.sin_port = htons(local_port);

    if (bind(sock_fd_, (const struct sockaddr *)&local_addr_, sizeof(local_addr_)) < 0)
    {
        // Make sure to close the socket, since the destructor won't be called
        close(sock_fd_);
        throw std::runtime_error("socket bind failed");
    }

    memset(&remote_addr_, 0, sizeof(remote_addr_));
    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_addr.s_addr = inet_addr(remote_ip);
    remote_addr_.sin_port = htons(remote_port);

    running_ = true;
    rx_thread_ = std::unique_ptr<std::thread>(new std::thread(&Lidar::rx_worker, this));
}

Lidar::~Lidar()
{
    running_ = false;
    rx_thread_->join();

    close(sock_fd_);
}

void Lidar::sync_time(uint32_t time_sec, uint32_t time_nsec)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);

        out_buffer_.clear();
        time_sec_ = time_sec;
        time_nsec_ = time_nsec;
        wait_for_cmd_ack_ = true;
    }

    TimeStampPacket packet{};
    packet.header.header[0] = FRAME_HEADER_BYTE_0;
    packet.header.header[1] = FRAME_HEADER_BYTE_1;
    packet.header.header[2] = FRAME_HEADER_BYTE_2;
    packet.header.header[3] = FRAME_HEADER_BYTE_3;
    packet.header.packet_type = TIME_STAMP_PACKET_TYPE;
    packet.header.packet_size = sizeof(TimeStampPacket);

    packet.data.sec = time_sec;
    packet.data.nsec = time_nsec;

    CRC32(TimeStampPacket);
    packet.tail.tail[0] = FRAME_TAIL_BYTE_0;
    packet.tail.tail[1] = FRAME_TAIL_BYTE_1;

    SEND_PACKET(TimeStampPacket);
}

void Lidar::set_work_mode(bool wide_fov, bool cloud_2d, bool disable_imu, bool use_serial, bool start_standby)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);

        out_buffer_.clear();
        wait_for_cmd_ack_ = true;
    }

    WorkModeConfigPacket packet{};
    packet.header.header[0] = FRAME_HEADER_BYTE_0;
    packet.header.header[1] = FRAME_HEADER_BYTE_1;
    packet.header.header[2] = FRAME_HEADER_BYTE_2;
    packet.header.header[3] = FRAME_HEADER_BYTE_3;
    packet.header.packet_type = WORK_MODE_CONFIG_PACKET_TYPE;
    packet.header.packet_size = sizeof(WorkModeConfigPacket);

    packet.data.mode = 0;
    packet.data.mode |= (wide_fov ? 1 : 0) << 0;
    packet.data.mode |= (cloud_2d ? 1 : 0) << 1;
    packet.data.mode |= (disable_imu ? 1 : 0) << 2;
    packet.data.mode |= (use_serial ? 1 : 0) << 3;
    packet.data.mode |= (start_standby ? 1 : 0) << 4;

    CRC32(WorkModeConfigPacket);
    packet.tail.tail[0] = FRAME_TAIL_BYTE_0;
    packet.tail.tail[1] = FRAME_TAIL_BYTE_1;

    SEND_PACKET(WorkModeConfigPacket);
}

L2Cloud Lidar::get_cloud()
{
    std::lock_guard<std::mutex> lock(mutex_);

    return out_buffer_.get_cloud();
}

L2Imu Lidar::get_imu()
{
    std::lock_guard<std::mutex> lock(mutex_);

    return out_buffer_.get_imu();
}