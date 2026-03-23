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

    void init() const;

    void setSpeed(int speed) const;

    void setAcceleration(int acceleration) const;

    void setHomePos() const;

    void run() const;

    void stop() const;

    void runToPosition(const int position) const;

    AccelStepper* getStepper() const;

private:
    AccelStepper* stepper_;

    const int EN_PIN_;
    const int DIR_PIN_;
    const int STEP_PIN_;
};

#endif //BEERCRATEGRIPPER_STEPPERMOTOR_HPP