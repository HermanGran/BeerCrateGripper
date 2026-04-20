//
// Created by Herman Hårstad Gran on 23/03/2026.
//
#include "Gripper/Gripper.hpp"
#include "Actuators/StepperMotor.hpp"
#include "Sensors/LimitSwitch.hpp"

Gripper::Gripper()
    : stepper_(5, 12, 11),
      limit_(4),
      current_(A7)
{}

void Gripper::init() {
    stepper_.init();
    limit_.init();
}

void Gripper::homing() {
    Serial.println("Homing...");
    auto s = stepper_.getStepper();
    s->setSpeed(-2000);

    //stepper_.getStepper()->moveTo(1000000);

    Serial.print("Waiting for limit switch...");
    int lastMillis = 0;
    while (!limit_.isPressed()) {
        if (lastMillis + 1000 < millis() ) {
            lastMillis = millis();
            Serial.print(".");
        }

        s->runSpeed();
    }



    Serial.println();
    Serial.println("Limit switch pressed!");

    s->setSpeed(400);
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

void Gripper::close() {
    stepper_.runToPosition(0);
    Serial.println("Closed");
}

void Gripper::open() {
    stepper_.runToPosition(3200 * 6);
    Serial.println("Opened");
}


