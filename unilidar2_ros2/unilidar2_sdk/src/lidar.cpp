#include <zlib.h>
#include "lidar.hpp"

void Lidar::buffer_packet(const DecodeRes &res)
{
    switch (res.header.packet_type)
    {
    case POINT_DATA_PACKET_TYPE:
        out_buffer_.add_points(reinterpret_cast<PointData *>(res.data.get()));
        break;
    case IMU_DATA_PACKET_TYPE:
    {
        auto imu = reinterpret_cast<ImuData *>(res.data.get());
        double diff = (imu->info.stamp.sec - last_imu.sec) + (imu->info.stamp.nsec - last_imu.nsec) / 1e9;
        std::cout << "Imu took " << (int)(diff * 1000) << " ms since last imu" << std::endl;
        last_imu = imu->info.stamp;
        out_buffer_.add_imu(imu);
        break;
    }

    case ACK_DATA_PACKET_TYPE:
    {
        auto ack = reinterpret_cast<AckData *>(res.data.get());
        expect_cmd_ack_ = false;
        std::cout << "ACK " << ack->packet_type << " " << ack->cmd_type << " " << ack->cmd_value << " " << ack->status << std::endl;
        break;
    }
    }
}

void Lidar::rx_worker()
{
    while (running_)
    {
        // Non-blocking receive to prevent the thread from being stuck when few packets are being sent.
        ssize_t n = recvfrom(sock_fd_, buffer_ + buffer_used_, sizeof(buffer_) - buffer_used_, MSG_DONTWAIT, nullptr, nullptr);

        if (n < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // No data available, sleep for a short time to avoid busy waiting
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            std::cerr << "recvfrom failed: " << strerror(errno) << std::endl;
            continue;
        }

        buffer_used_ += n;
        uint8_t *decode_ptr_ = buffer_;

        try
        {
            while (buffer_used_ > 0)
            {
                DecodeRes res = decode_packet(decode_ptr_, buffer_used_);

                if (res.bytes_parsed == 0)
                {
                    // We need another packet to decode.
                    break;
                }

                decode_ptr_ += res.bytes_parsed;
                buffer_used_ -= res.bytes_parsed;

                std::lock_guard<std::mutex> lock(mutex_);

                // Ignore non-ack packets while waiting for an ack
                if (wait_for_cmd_ack_)
                {
                    // TODO: Check that we have gotten the right ack packet for the command we sent
                    if (res.header.packet_type != ACK_DATA_PACKET_TYPE)
                    {
                        continue;
                    }
                }

                buffer_packet(res);
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error decoding packet: " << e.what() << std::endl;
            // If we can't decode the packet, we can't easily know how many bytes to skip, so we have to discard the rest of the buffer.
            buffer_used_ = 0;
            continue;
        }

        // Move any remaining bytes to the front of the buffer for the next iteration
        std::memmove(buffer_, decode_ptr_, buffer_used_);
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

bool Lidar::wait_for_ack(int64_t timeout_ms)
{
    wait_for_cmd_ack_ = true;
    auto start_time = std::chrono::steady_clock::now();

    while (expect_cmd_ack_)
    {
        auto elapsed_time = std::chrono::steady_clock::now() - start_time;

        if (elapsed_time.count() > timeout_ms * 1000000)
        {
            return false;
        }

        // Sleep for a short time to avoid busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    wait_for_cmd_ack_ = false;
    return true;
}

void Lidar::sync_time(uint32_t time_sec, uint32_t time_nsec)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);

        out_buffer_.clear();
        expect_cmd_ack_ = true;
    }

    TimeStampPacket packet{};
    SET_FRAME_HEADER(TimeStampPacket, TIME_STAMP_PACKET_TYPE);

    packet.data.sec = time_sec;
    packet.data.nsec = time_nsec;

    CRC32(TimeStamp);
    SET_FRAME_TAIL;

    SEND_PACKET(TimeStampPacket);
}

void Lidar::set_work_mode(bool wide_fov)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);

        out_buffer_.clear();
        expect_cmd_ack_ = true;
    }

    WorkModeConfigPacket packet{};
    SET_FRAME_HEADER(WorkModeConfigPacket, WORK_MODE_CONFIG_PACKET_TYPE);

    packet.data.mode = 0;
    packet.data.mode |= (wide_fov ? 1 : 0) << 0;

    CRC32(WorkModeConfig);
    SET_FRAME_TAIL;

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