#ifndef CHCNAV_UDP_COMMON_HPP_
#define CHCNAV_UDP_COMMON_HPP_

#include "device_connector.hpp"

#include <cstring>
#include <iostream>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

class udp_common final : public hc__device_connector
{
private:
    int status = -1;              // 连接状态。 -1 未连接，1 已连接
    int port;                     // 端口号
    int socketfd = -1;              // socket fd
    struct sockaddr_in sock_addr{}; // ip信息结构体
    bool port_valid = false;

public:
    udp_common(int port) : hc__device_connector()
    {
        this->port = port;

        this->sock_addr.sin_family = AF_INET;
        this->sock_addr.sin_port = htons(this->port);
        this->sock_addr.sin_addr.s_addr = inet_addr("0.0.0.0");
        port_valid = this->port > 0 && this->port <= 65535;
    }

    int connect(void) override
    {
        this->disconnect();
        if (!port_valid)
        {
            return -1;
        }

        // 建立套接字
        this->socketfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (this->socketfd < 0)
        {
            return -1;
        }
        if (bind(this->socketfd, (struct sockaddr *)&this->sock_addr, sizeof(this->sock_addr)) == -1)
        {
            this->disconnect();
            return -1;
        }
        else
        {
            const int flags = fcntl(this->socketfd, F_GETFL, 0);
            if (flags < 0 || fcntl(this->socketfd, F_SETFL, flags | O_NONBLOCK) < 0)
            {
                this->disconnect();
                return -1;
            }
            this->status = 1;
            return 0;
        }
    }

    int write(const char *data, unsigned int len) override
    {
        if (data == nullptr || len == 0U)
        {
            return 0;
        }
        if (this->status != 1 && this->connect() != 0)
        {
            sleep(1);
            return -1;
        }

        int write_bytes = sendto(this->socketfd, data, len, 0, (struct sockaddr *)(&this->sock_addr), sizeof(this->sock_addr));
        if (write_bytes == -1)
        {
            printf("socker [%d] send failed\n", this->socketfd);
            this->disconnect();
            return -1;
        }

        return write_bytes;
    }

    int read(char *data, unsigned int maxsize) override
    {
        if (data == nullptr || maxsize == 0U)
        {
            return 0;
        }
        if (this->status != 1 && this->connect() != 0)
        {
            sleep(1);
            return -1;
        }

        socklen_t len = sizeof(this->sock_addr);
        int read_bytes = recvfrom(this->socketfd, data, maxsize, 0, (struct sockaddr *)(&this->sock_addr), &len);
        if (read_bytes <= 0)
        {
            if (read_bytes < 0 && (errno == EWOULDBLOCK || errno == EAGAIN))
            {
                errno = 0;
            }
            else
            {
                printf("socker [%d] read failed\n", this->socketfd);
                this->disconnect();
                return -1;
            }
        }

        return read_bytes;
    }

    int disconnect(void) override
    {
        if (socketfd >= 0)
        {
            close(socketfd);
            socketfd = -1;
        }
        this->status = -1;

        return 0;
    }

    ~udp_common() override { disconnect(); }
};

#endif  // CHCNAV_UDP_COMMON_HPP_
