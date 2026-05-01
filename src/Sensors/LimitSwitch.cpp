//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#include "Sensors/LimitSwitch.hpp"
#include <Arduino.h>

LimitSwitch::LimitSwitch(const int pin) : pin_(pin) {}

void LimitSwitch::init() const noexcept {
    pinMode(pin_, INPUT_PULLUP);
}

bool LimitSwitch::isPressed() const noexcept {
    return digitalRead(pin_) != LOW;
}