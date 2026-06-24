#include <zlib.h>
#include "lidar.hpp"

void Lidar::clear_packet_buffer()
{
    // Pointers in the packet buffer need to be manually freed
    while (!packet_buffer_.empty())
    {
        delete[] packet_buffer_.front().data;
        packet_buffer_.pop();
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

                BufferedPacket packet;
                packet.packet_type = res.header.packet_type;
                packet.data = res.data; // Ownership of the data pointer is transferred to the packet buffer
                packet_buffer_.push(packet);

                if (packet_buffer_.size() > PACKET_BUFFER_CAPACITY)
                {
                    delete[] packet_buffer_.front().data;
                    packet_buffer_.pop();
                }
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

    clear_packet_buffer();
    close(sock_fd_);
}

void Lidar::sync_time(uint32_t time_sec, uint32_t time_nsec)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);

        clear_packet_buffer();
        time_sec_ = time_sec;
        time_nsec_ = time_nsec;
        wait_for_cmd_ack_ = true;
    }

    TimeStampPacket packet;
    packet.header.packet_type = TIME_STAMP_PACKET_TYPE;
    packet.header.packet_size = sizeof(TimeStampPacket);

    packet.data.sec = time_sec;
    packet.data.nsec = time_nsec;

    CRC32(TimeStampPacket);

    SEND_PACKET(TimeStampPacket);
}

void Lidar::set_work_mode(bool wide_fov, bool cloud_2d, bool disable_imu, bool use_serial, bool start_standby)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);

        clear_packet_buffer();
        wait_for_cmd_ack_ = true;
    }

    WorkModeConfigPacket packet;
    packet.header.packet_type = WORK_MODE_CONFIG_PACKET_TYPE;
    packet.header.packet_size = sizeof(WorkModeConfigPacket);

    packet.data.mode = 0;
    packet.data.mode |= (wide_fov ? 1 : 0) << 0;
    packet.data.mode |= (cloud_2d ? 1 : 0) << 1;
    packet.data.mode |= (disable_imu ? 1 : 0) << 2;
    packet.data.mode |= (use_serial ? 1 : 0) << 3;
    packet.data.mode |= (start_standby ? 1 : 0) << 4;

    CRC32(WorkModeConfigPacket);

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

// int Lidar::has_data()
// {
//     std::lock_guard<std::mutex> lock(mutex_);

//     if (packet_buffer_.empty())
//     {
//         return 0;
//     }

//     return packet_buffer_.front().packet_type;
// }

// void Lidar::ignore_data()
// {
//     std::lock_guard<std::mutex> lock(mutex_);

//     if (!packet_buffer_.empty())
//     {
//         delete[] packet_buffer_.front().data;
//         packet_buffer_.pop();
//     }
// }

// PointData Lidar::get_point_data()
// {
//     std::lock_guard<std::mutex> lock(mutex_);

//     if (packet_buffer_.empty())
//     {
//         throw std::runtime_error("No data available");
//     }

//     BufferedPacket packet = packet_buffer_.front();

//     if (packet.packet_type != POINT_DATA_PACKET_TYPE)
//     {
//         throw std::runtime_error("Packet type is not PointDataPacket");
//     }

//     packet_buffer_.pop();
//     PointData out;
//     out = *reinterpret_cast<PointData *>(packet.data);
//     delete[] packet.data;

//     return out;
// }

// ImuData Lidar::get_imu_data()
// {
//     std::lock_guard<std::mutex> lock(mutex_);

//     if (packet_buffer_.empty())
//     {
//         throw std::runtime_error("No data available");
//     }

//     BufferedPacket packet = packet_buffer_.front();

//     if (packet.packet_type != IMU_DATA_PACKET_TYPE)
//     {
//         throw std::runtime_error("Packet type is not ImuDataPacket");
//     }

//     packet_buffer_.pop();
//     ImuData out;
//     out = *reinterpret_cast<ImuData *>(packet.data);
//     delete[] packet.data;

//     return out;
// }
