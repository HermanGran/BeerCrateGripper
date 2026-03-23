//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#include "Sensors/LimitSwitch.hpp"

LimitSwitch::LimitSwitch(const int pin) : pin_(pin) {}

void LimitSwitch::init() const {
    pinMode(pin_, INPUT_PULLUP);
}

bool LimitSwitch::isPressed() const {
    return digitalRead(pin_) != LOW;
}