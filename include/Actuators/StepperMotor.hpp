//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#ifndef BEERCRATEGRIPPER_STEPPERMOTOR_HPP
#define BEERCRATEGRIPPER_STEPPERMOTOR_HPP

#include <Arduino.h>
#include <AccelStepper.h>
#include <Sensors/CurrentSensor.hpp>

class StepperMotor {
public:
    StepperMotor(int EN_PIN, int DIR_PIN, int STEP_PIN, CurrentSensor& currentSensor);

    void init();

    void setSpeed(int speed);

    void setAcceleration(int acceleration);

    void setHomePos();

    void run();

    void stop();

    void runToPosition(const int position);

    AccelStepper* getStepper();

    bool isRunning() const;

private:
    AccelStepper stepper_;
    CurrentSensor& current_;

    const int EN_PIN_;
    const int DIR_PIN_;
    const int STEP_PIN_;

    volatile bool running_ = false;
};

#endif //BEERCRATEGRIPPER_STEPPERMOTOR_HPP