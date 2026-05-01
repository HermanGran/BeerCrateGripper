//
// Created by Herman Hårstad Gran on 23/04/2026.
//
#include "Debug/UDPLogger.hpp"

// Initialization function
void UDPLogger::init(const IPAddress& ip, const uint16_t port) {
    pcIP_ = ip;
    port_ = port;
    udp_.begin(port_);
}

// Logger function, send the message to the predefined IP
void UDPLogger::log(const char *msg) {
    udp_.beginPacket(pcIP_, port_);
    udp_.write(reinterpret_cast<const uint8_t *>(msg), strlen(msg));
    udp_.write(reinterpret_cast<const uint8_t *>("\n"), 1);
    udp_.endPacket();
}

// Log format. Can be used with variables
void UDPLogger::logf(const char *format, ...) {
    char buf[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    log(buf);
}
