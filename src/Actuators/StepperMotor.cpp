//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#include "Actuators/StepperMotor.hpp"

StepperMotor::StepperMotor(const int EN_PIN, const int DIR_PIN, const int STEP_PIN)
    : EN_PIN_(EN_PIN), DIR_PIN_(DIR_PIN), STEP_PIN_(STEP_PIN) {
    stepper_ = new AccelStepper(AccelStepper::DRIVER, STEP_PIN_, DIR_PIN_);
}

void StepperMotor::init() const {
    pinMode(EN_PIN_, OUTPUT);
    digitalWrite(EN_PIN_, LOW);   // TMC2208 enabled
    stepper_->setMaxSpeed(6000);      // steps per second
    stepper_->setAcceleration(2000);  // steps per second^2
}

void StepperMotor::setSpeed(const int speed) const {
    stepper_->setMaxSpeed(speed);
}

void StepperMotor::setAcceleration(const int acceleration) const {
    stepper_->setAcceleration(acceleration);
}


void StepperMotor::setHomePos() const {
    stepper_->setCurrentPosition(0);
}

AccelStepper* StepperMotor::getStepper() const {
    return stepper_;
}

void StepperMotor::run() const {
    stepper_->run();
}

void StepperMotor::stop() const {
    stepper_->stop();
}

void StepperMotor::runToPosition(const int position) const {
    stepper_->moveTo(position);
    while (stepper_->distanceToGo() != 0) {
        stepper_->run();
    }
}
