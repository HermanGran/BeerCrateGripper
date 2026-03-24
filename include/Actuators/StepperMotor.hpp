//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#ifndef BEERCRATEGRIPPER_STEPPERMOTOR_HPP
#define BEERCRATEGRIPPER_STEPPERMOTOR_HPP

#include <Arduino.h>
#include <AccelStepper.h>

class StepperMotor {
public:
    StepperMotor(const int EN_PIN, const int DIR_PIN, const int STEP_PIN);

    void init();

    void setSpeed(int speed);

    void setAcceleration(int acceleration);

    void setHomePos();

    void run();

    void stop();

    void runToPosition(const int position);

    AccelStepper* getStepper();

private:
    AccelStepper stepper_;

    const int EN_PIN_;
    const int DIR_PIN_;
    const int STEP_PIN_;
};

#endif //BEERCRATEGRIPPER_STEPPERMOTOR_HPP