#ifndef CHCNAV_TCP_COMMON_HPP_
#define CHCNAV_TCP_COMMON_HPP_

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

class tcp_common final : public hc__device_connector
{
private:
    int status = -1;                // 连接状态。 -1 未连接，1 已连接
    std::string host;               // ip
    int port;                       // 端口号
    int socketfd = -1;                // socket fd
    struct sockaddr_in client_addr{}; // 客户端信息结构体
    bool address_valid = false;

public:
    tcp_common(std::string host, int port) : hc__device_connector()
    {
        this->host = host;
        this->port = port;

        this->client_addr.sin_family = AF_INET;
        this->client_addr.sin_port = htons(this->port);
        address_valid =
            inet_pton(AF_INET, this->host.c_str(), &this->client_addr.sin_addr) == 1 &&
            this->port > 0 && this->port <= 65535;
    }

    int connect(void) override
    {
        this->disconnect();
        if (!address_valid)
        {
            fprintf(stderr, "invalid tcp endpoint [%s:%d]\n", host.c_str(), port);
            return -1;
        }

        // 建立套接字
        this->socketfd = socket(AF_INET, SOCK_STREAM, 0);
        if (this->socketfd < 0)
        {
            return -1;
        }

        int ret = ::connect(this->socketfd, (struct sockaddr *)&this->client_addr, sizeof(struct sockaddr_in));
        if (ret < 0)
        {
            this->disconnect();
            fprintf(stderr, "tcp connect fail!\n");
            return -1;
        }

        fprintf(stdout, "tcp connect success!\n");
        this->status = 1;

        int flag = fcntl(this->socketfd, F_GETFL, 0);
        if (flag < 0 || fcntl(this->socketfd, F_SETFL, flag | O_NONBLOCK) < 0)
        {
            this->disconnect();
            return -1;
        }

        return 0;
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

        int write_bytes = ::write(this->socketfd, data, len);
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

        int read_bytes = ::read(this->socketfd, data, maxsize);
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

    ~tcp_common() override { disconnect(); }
};

#endif  // CHCNAV_TCP_COMMON_HPP_
