//
// Created by Herman Hårstad Gran on 24/03/2026.
//

#ifndef BEERCRATEGRIPPER_SERIALCOMMANDHANDLER_HPP
#define BEERCRATEGRIPPER_SERIALCOMMANDHANDLER_HPP

#include <Arduino.h>
#include "Gripper/Gripper.hpp"

class SerialCommandHandler {
public:
    explicit SerialCommandHandler(Gripper& gripper);

    void init(unsigned long baud = 115200);

    void update();

private:
    Gripper& gripper_;

    String buffer_;

    void processCommand(const String& cmd);

};

#endif //BEERCRATEGRIPPER_SERIALCOMMANDHANDLER_HPP