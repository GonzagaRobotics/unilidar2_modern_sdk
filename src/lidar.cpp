#include "lidar.hpp"

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

        try
        {
            DecodeRes res = decode_packet(buffer_, n);

            std::lock_guard<std::mutex> lock(mutex_);
            BufferedPacket packet;
            packet.packet_type = res.header.packet_type;
            packet.data = res.data; // Ownership of the data pointer is transferred to the packet buffer
            packet_buffer_.push(packet);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error decoding packet: " << e.what() << std::endl;
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
    rx_thread_ = std::thread(&Lidar::rx_worker, this);
    rx_thread_.detach();
}

Lidar::~Lidar()
{
    running_ = false;
    rx_thread_.join();

    // Pointers in the packet buffer need to be manually freed
    while (!packet_buffer_.empty())
    {
        delete[] packet_buffer_.front().data;
        packet_buffer_.pop();
    }

    close(sock_fd_);
}

int Lidar::has_data()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (packet_buffer_.empty())
    {
        return 0;
    }

    return packet_buffer_.front().packet_type;
}

void Lidar::ignore_data()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!packet_buffer_.empty())
    {
        delete[] packet_buffer_.front().data;
        packet_buffer_.pop();
    }
}

PointData Lidar::get_point_data()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (packet_buffer_.empty())
    {
        throw std::runtime_error("No data available");
    }

    BufferedPacket packet = packet_buffer_.front();

    if (packet.packet_type != POINT_DATA_PACKET_TYPE)
    {
        throw std::runtime_error("Packet type is not PointDataPacket");
    }

    packet_buffer_.pop();
    PointData out;
    out = *reinterpret_cast<PointData *>(packet.data);
    delete[] packet.data;

    return out;
}

ImuData Lidar::get_imu_data()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (packet_buffer_.empty())
    {
        throw std::runtime_error("No data available");
    }

    BufferedPacket packet = packet_buffer_.front();

    if (packet.packet_type != IMU_DATA_PACKET_TYPE)
    {
        throw std::runtime_error("Packet type is not ImuDataPacket");
    }

    packet_buffer_.pop();
    ImuData out;
    out = *reinterpret_cast<ImuData *>(packet.data);
    delete[] packet.data;

    return out;
}
