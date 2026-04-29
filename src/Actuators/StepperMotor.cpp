//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#include "Actuators/StepperMotor.hpp"

StepperMotor::StepperMotor(const int EN_PIN, const int DIR_PIN, const int STEP_PIN, CurrentSensor& currentSensor)
    :   stepper_(AccelStepper::DRIVER, STEP_PIN, DIR_PIN),
        current_(currentSensor),
        EN_PIN_(EN_PIN),
        DIR_PIN_(DIR_PIN),
        STEP_PIN_(STEP_PIN)
{}

void StepperMotor::init() {
    pinMode(EN_PIN_, OUTPUT);
    digitalWrite(EN_PIN_, LOW);   // TMC2208 enabled
    stepper_.setMaxSpeed(4000);      // steps per second
    stepper_.setAcceleration(2000);  // steps per second^2
    stepper_.setEnablePin(EN_PIN_);
    stepper_.setPinsInverted(false, false, true);
    stepper_.disableOutputs();
}

void StepperMotor::setSpeed(const int speed) {
    stepper_.setMaxSpeed(speed);
}

void StepperMotor::setAcceleration(const int acceleration) {
    stepper_.setAcceleration(acceleration);
}


void StepperMotor::setHomePos() {
    stepper_.setCurrentPosition(0);
}

AccelStepper* StepperMotor::getAccelStepper() {
    return &stepper_;
}


void StepperMotor::stop() {
    stepper_.stop();
}

void StepperMotor::runToPosition(const int position) {
    stepper_.enableOutputs();
    stepper_.moveTo(position);
    running_ = true;
}

void StepperMotor::run() {
    if (running_) {
        stepper_.run();
        if (stepper_.distanceToGo() == 0) {
            running_ = false;
            stepper_.disableOutputs();
        }
    }
}

bool StepperMotor::isRunning() const {
    return running_;
}

