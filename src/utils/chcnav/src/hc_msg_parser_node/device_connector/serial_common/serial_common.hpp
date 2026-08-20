#ifndef CHCNAV_SERIAL_COMMON_HPP_
#define CHCNAV_SERIAL_COMMON_HPP_

#include "device_connector.hpp"
#include "serial/serial.h"
#include <cstdint>
#include <exception>
#include <memory>
#include <pthread.h>
#include <string>
#include <termios.h>
#include <utility>

class serial_common final : public hc__device_connector
{
private:
    std::string port = "NULL";          // serial port
    std::unique_ptr<serial::Serial> serial_device;
    int status = -1;                    // -1 not connected. 1 connected
    uint32_t baudrate = 115200;         // bound rate. 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800
    serial::bytesize_t bytesize = serial::eightbits;
    serial::parity_t parity = serial::parity_none;
    serial::stopbits_t stopbits = serial::stopbits_one;
    bool configured = false;

public:
    /**
     * @brief default Construct
     * */
    serial_common(void) : hc__device_connector()
    {
        this->status = -1;
    };

    /**
     * @brief serial_common object
     *
     * @param port      // serial port
     * @param baudrate     // bound rate. 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800
     * @param bits      // data bits,such 7/8 etc
     * @param stop      // stop bits, 1/2 etc
     * @param parity    // checkout bits. None Odd Even.
     *
     * @note this constructor will not open the serial port.
     */
    serial_common(std::string port, int baudrate, short bits, short stop, std::string parity) : hc__device_connector()
    {
        this->status = -1;
        configured = (this->set(port, baudrate, bits, stop, parity) == 0);
    }

    int set(std::string port, int baudrate, short bits, short stop, std::string parity)
    {
        int baudrate_attr[] = {300, 600, 1200, 2400, 4800, 9600,
                            19200, 38400, 57600, 115200, 230400, 460800, 921600};

        // disconnect first if already connnect
        if (this->status == 1)
            this->disconnect();

        // try access the port
        if (access(port.c_str(), F_OK) != 0)
        {
            fprintf(stderr, "access [%s] failed\n", port.c_str());
            return -1;
        }
        this->port = port;

        // set baudrate
        bool baudrate_supported = false;
        for (std::size_t temp_i = 0;
             temp_i < sizeof(baudrate_attr) / sizeof(baudrate_attr[0]);
             ++temp_i)
        {
            if (baudrate == baudrate_attr[temp_i])
            {
                this->baudrate = baudrate_attr[temp_i];
                baudrate_supported = true;
                break;
            }
        }
        if (!baudrate_supported)
        {
            fprintf(stderr, "error baudrate\n");
            return -1;
        }

        // set databits
        switch (bits)
        {
            case 5:
                this->bytesize = serial::fivebits;
                break;
            case 6:
                this->bytesize = serial::sixbits;
                break;
            case 7:
                this->bytesize = serial::sevenbits;
                break;
            case 8:
                this->bytesize = serial::eightbits;
                break;
            default:
                fprintf(stderr, "error databits\n");
                return -1;
        }

        // set stopbits
        switch (stop)
        {
            case 1:
                this->stopbits = serial::stopbits_one;
                break;
            case 2:
                this->stopbits = serial::stopbits_two;
                break;
            default:
                fprintf(stderr, "error stopbits\n");
                return -1;
        }

        // set parity
        if (parity.compare("None") == 0)
        {
            this->parity = serial::parity_none;
        }
        else if (parity.compare("Odd") == 0)
        {
            this->parity = serial::parity_odd;
        }
        else if (parity.compare("Even") == 0)
        {
            this->parity = serial::parity_even;
        }
        else if (parity.compare("Mark") == 0)
        {
            this->parity = serial::parity_mark;
        }
        else if (parity.compare("Space") == 0)
        {
            this->parity = serial::parity_space;
        }
        else
        {
            fprintf(stderr, "error parity [%s]\n", parity.c_str());
            return -1;
        }

        configured = true;
        return 0;
    }

    /**
     * @brief open a serial port
     *
     * @note if a object call this function which has opened a serial port, this func will do nothing.
     *
     * @return int < 0 failed. 0 success. 1 already opened.
     */
    int connect(void) override
    {
        if (!configured)
        {
            return -1;
        }

        disconnect();
        try
        {
            std::unique_ptr<serial::Serial> candidate(new serial::Serial(
                this->port, this->baudrate, serial::Timeout::simpleTimeout(100),
                this->bytesize, this->parity, this->stopbits,
                serial::flowcontrol_none));
            if (!candidate->isOpen())
            {
                return -1;
            }
            candidate->setRTS(true);
            candidate->setDTR(true);
            serial_device = std::move(candidate);
            status = 1;
            return 0;
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "serial connect failed: %s\n", e.what());
            serial_device.reset();
            status = -1;
            return -1;
        }
    }

    /**
     * @brief write data to device
     *
     * @param data data block
     * @param len length of data
     * @return int nums of written. -1 means write fail.
     */
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

        return this->serial_device->write(
            reinterpret_cast<const unsigned char *>(data), len);
    }

    /**
     * @brief read data from device
     *
     * @param data storage of data
     * @param maxsize max size of the data memory
     * @return int nums of read. -1 means failed.
     */
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
        return this->serial_device->read(
            reinterpret_cast<unsigned char *>(data), maxsize);
    }

    /**
     * @brief close this object
     *
     * @return int 0 means succeed. -1 failed.
     */
    int disconnect(void) override
    {
        this->status = -1;

        if (this->serial_device && this->serial_device->isOpen())
            this->serial_device->close();
        this->serial_device.reset();

        return 0;
    }

    /**
     * @brief Destroy object and close serial
     */
    ~serial_common() override { this->disconnect(); }
};

#endif  // CHCNAV_SERIAL_COMMON_HPP_
