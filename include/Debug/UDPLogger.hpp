//
// Created by Herman Hårstad Gran on 23/04/2026.
//

#ifndef BEERCRATEGRIPPER_UDPLOGGER_HPP
#define BEERCRATEGRIPPER_UDPLOGGER_HPP

#include <WiFiUdp.h>
#include <stdio.h>

class UDPLogger {
public:
    WiFiUDP udp;
    IPAddress pc_ip;
    uint16_t port;

    void init(IPAddress ip, uint16_t p);

    void log(const char* msg);

    void logf(const char* format, ...);

};

#endif //BEERCRATEGRIPPER_UDPLOGGER_HPP