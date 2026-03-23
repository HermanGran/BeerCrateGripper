//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#ifndef BEERCRATEGRIPPER_GRIPPER_HPP
#define BEERCRATEGRIPPER_GRIPPER_HPP

#include <Arduino.h>
#include <Actuators/StepperMotor.hpp>
#include <Sensors/LimitSwitch.hpp>
#include <Sensors/CurrentSensor.hpp>

class Gripper {
public:
    Gripper();
    void init() const;
    void homing() const;
    void open() const;
    void close() const;
    bool isClosed(); // To be added later, must include the current sensor to sense latching

private:
    StepperMotor* stepper_;
    LimitSwitch* limit_;
    CurrentSensor* current_;
};

#endif //BEERCRATEGRIPPER_GRIPPER_HPP