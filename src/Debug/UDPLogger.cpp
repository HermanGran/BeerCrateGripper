//
// Created by Herman Hårstad Gran on 23/04/2026.
//
#include "Debug/UDPLogger.hpp"


void UDPLogger::init(IPAddress ip, uint16_t p) {
    pc_ip = ip;
    port = p;
    udp.begin(port);
}

void UDPLogger::log(const char *msg) {
    udp.beginPacket(pc_ip, port);
    udp.write((const uint8_t*)msg, strlen(msg));
    udp.write((const uint8_t*)"\n", 1);
    udp.endPacket();
}

void UDPLogger::logf(const char *format, ...) {
    char buf[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    log(buf);
}
