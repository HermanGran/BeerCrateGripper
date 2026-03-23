//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#ifndef BEERCRATEGRIPPER_LIMITSWITCH_HPP
#define BEERCRATEGRIPPER_LIMITSWITCH_HPP

#include <Arduino.h>

class LimitSwitch {
public:
    explicit LimitSwitch(int pin);

    void init() const;

    bool isPressed() const;

private:
    int pin_;
};

#endif //BEERCRATEGRIPPER_LIMITSWITCH_HPP