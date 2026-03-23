#include <Arduino.h>
// Arduino Nano ESP32 + TMC2208 in STEP/DIR mode
#include <AccelStepper.h>

const int STEP_PIN = 5;
const int DIR_PIN  = 4;
const int EN_PIN   = 6;


// DRIVER mode = STEP + DIR
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);   // TMC2208 enabled

    stepper.setMaxSpeed(6000);      // steps per second
    stepper.setAcceleration(2000);  // steps per second^2
}

void loop() {
    stepper.moveTo(-3200*10);   // one rev if 200-step motor at 1/16 => 3200 microsteps
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }

    delay(500);

    stepper.moveTo(0);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }

    delay(500);
}