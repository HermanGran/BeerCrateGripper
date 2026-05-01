//
// Created by Herman Hårstad Gran on 23/04/2026.
//

#ifndef BEERCRATEGRIPPER_UDPLOGGER_HPP
#define BEERCRATEGRIPPER_UDPLOGGER_HPP

#include <WiFiUdp.h>

/**
 * @class UDPLogger
 * @brief A simple logger class that sends log messages over a UDP connection.
 *
 * This class provides methods to initialize a UDP connection and transmit log
 * messages to a specified IP address and port. This avoids the need for wired connection with the
 * Arduino Nano ESP32.
 */
class UDPLogger {
public:
    /**
     * @brief Initialization function
     *
     * @note This function needs to be called before the usage of the logger class
     *
     * @param ip Address to send the UDP logs
     * @param port To send the UDP packages to
     */
    void init(const IPAddress& ip, uint16_t port);

    /**
     * Sends msg via UDP to the predefined IP Address and port
     *
     * @param msg const char message
     */
    void log(const char* msg);

    /**
     * @brief Enable formatted messages to be sent
     *
     * This function creates a msg with variables to be sent via log function
     *
     * @param format Formatted msg
     * @param ... variables
     */
    void logf(const char* format, ...);

private:
    WiFiUDP udp_;
    IPAddress pcIP_;
    uint16_t port_ = 0;
};

#endif //BEERCRATEGRIPPER_UDPLOGGER_HPP