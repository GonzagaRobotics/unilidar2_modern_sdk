#pragma once

#include <string.h>
#include <stdexcept>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "unilidar2_sdk/source.hpp"

namespace unilidar2
{
    class EnetSource : public Source
    {
    private:
        int sock_fd_;
        struct sockaddr_in local_addr_;
        struct sockaddr_in remote_addr_;

    public:
        EnetSource(const std::string &local_ip, uint16_t local_port, const std::string &remote_ip, uint16_t remote_port);
        ~EnetSource();

        size_t get_data(uint8_t *buffer, size_t buffer_size) override;

        void send_data(const uint8_t *data, size_t size) override;
    };
}