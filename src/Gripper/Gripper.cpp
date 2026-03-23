//
// Created by Herman Hårstad Gran on 23/03/2026.
//
#include "Gripper/Gripper.hpp"
#include "Actuators/StepperMotor.hpp"
#include "Sensors/LimitSwitch.hpp"

Gripper::Gripper() {
    stepper_ = new StepperMotor(5, 4, 6);
    limit_ = new LimitSwitch(10); // Må endre pin
    current_ = new CurrentSensor(13); // Må endre pin
}
void Gripper::init() const {
    stepper_->init();
    limit_->init();
}

void Gripper::homing() const {
    stepper_->setSpeed(800);
    stepper_->setAcceleration(400);

    while (limit_->isPressed()) {
        stepper_->runToPosition(stepper_->getStepper()->currentPosition() + 100);
    }

    stepper_->stop();
    while (stepper_->getStepper()->isRunning()) {
        stepper_->run();
    }

    stepper_->setHomePos();
    stepper_->runToPosition(-100);

    stepper_->setSpeed(6000);
    stepper_->setAcceleration(2000);
}

void Gripper::open() const {
    stepper_->runToPosition(-100);
}

void Gripper::close() const {
    stepper_->runToPosition(-3200 * 10);
}


