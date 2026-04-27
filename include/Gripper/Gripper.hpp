//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#ifndef BEERCRATEGRIPPER_GRIPPER_HPP
#define BEERCRATEGRIPPER_GRIPPER_HPP

#include <Arduino.h>
#include <Actuators/StepperMotor.hpp>
#include <Sensors/LimitSwitch.hpp>
#include <Sensors/CurrentSensor.hpp>

#define GRIPPER_HOME 0
#define GRIPPER_LATCH 1
#define GRIPPER_RELEASE 2

class Gripper {
public:
    Gripper();
    void init();
    void homing();
    void open();
    void close();
    bool isClosed(); // To be added later, must include the current sensor to sense latching

    StepperMotor& getStepper();

    CurrentSensor& getCurrentSensor();
    TaskHandle_t callerTaskHandle_ = NULL;
    volatile bool tasksRunning_ = false;
private:
    void moveToPosition(int position);

    StepperMotor stepper_;
    LimitSwitch limit_;
    CurrentSensor current_;
};

#endif //BEERCRATEGRIPPER_GRIPPER_HPP