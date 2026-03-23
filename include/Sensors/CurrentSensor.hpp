//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#ifndef BEERCRATEGRIPPER_CURRENTSENSOR_HPP
#define BEERCRATEGRIPPER_CURRENTSENSOR_HPP

#include <Arduino.h>

class CurrentSensor {
public:
    CurrentSensor(int pin);

private:
    int pin_;
};

#endif //BEERCRATEGRIPPER_CURRENTSENSOR_HPP