//
// Created by Herman Hårstad Gran on 23/03/2026.
//
#include "Gripper/Gripper.hpp"
#include "Actuators/StepperMotor.hpp"
#include "Sensors/LimitSwitch.hpp"

Gripper::Gripper()
    : stepper_(6, 4, 5),
      limit_(10),
      current_(13)
{}

void Gripper::init() {
    stepper_.init();
    limit_.init();
}

void Gripper::homing() {
    Serial.println("Homing...");
    auto s = stepper_.getStepper();
    s->setSpeed(2000);

    //stepper_.getStepper()->moveTo(1000000);

    Serial.print("Waiting for limit switch...");
    while (!limit_.isPressed()) {
        if (millis() % 1000 == 0) {
            Serial.print(".");
        }

        s->runSpeed();
    }



    Serial.println();
    Serial.println("Limit switch pressed!");

    s->setSpeed(-400);
    while (limit_.isPressed()) {
        if (millis() % 1000 == 0) {
            Serial.print(".");
        }

        s->runSpeed();
    }

    stepper_.setHomePos();
    //stepper_.runToPosition(-100);

    Serial.println("Homing done!");
}

void Gripper::open() {
    stepper_.runToPosition(0);
}

void Gripper::close() {
    stepper_.runToPosition(-3200 * 16);
}


