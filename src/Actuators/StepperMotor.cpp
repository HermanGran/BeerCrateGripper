//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#include "Actuators/StepperMotor.hpp"

// Constructor takes in pins for the motor driver and the current sensor
StepperMotor::StepperMotor(const int8_t enPin, const int8_t dirPin, const int8_t stepPin, const int8_t ms1Pin, const int8_t ms2Pin)
    :   stepper_(AccelStepper::DRIVER, stepPin, dirPin),
        enPin_(enPin),
        dirPin_(dirPin),
        stepPin_(stepPin),
        ms1Pin_(ms1Pin),
        ms2Pin_(ms2Pin)
{}

// Initialization function, initializes the pins for the motor driver and sets the speeds and acceleration of the motor.
void StepperMotor::init() {
    pinMode(enPin_, OUTPUT);
    pinMode(ms1Pin_, OUTPUT);
    pinMode(ms2Pin_, OUTPUT);
    digitalWrite(dirPin_, LOW);   // TMC2208 enabled
    stepper_.setMaxSpeed(4000);      // steps per second
    stepper_.setAcceleration(2000);  // steps per second^2
    stepper_.setEnablePin(enPin_);
    stepper_.setPinsInverted(false, false, true);
    stepper_.disableOutputs();
}

// Sets the max speed of the motor
void StepperMotor::setSpeed(const int speed) {
    stepper_.setMaxSpeed(speed);
}

// Sets the acceleration of the motor
void StepperMotor::setAcceleration(const int acceleration) {
    stepper_.setAcceleration(acceleration);
}

// Sets the current position of the motor as the new home
void StepperMotor::setHomePos() {
    stepper_.setCurrentPosition(0);
}

// Returns a reference to the accelStepper object
AccelStepper* StepperMotor::getAccelStepper() {
    return &stepper_;
}

// Stops the motor
void StepperMotor::stop() {
    stepper_.stop();
}

// Runs to a position in number of steps, the number of steps is stored internally
void StepperMotor::runToPosition(const int position) {
    stepper_.enableOutputs();
    stepper_.moveTo(position);
    running_ = true;
}

// Runs loop function. Must be called in a loop
void StepperMotor::run() {
    if (running_) {
        stepper_.run();
        if (stepper_.distanceToGo() == 0) {
            running_ = false;
            stepper_.disableOutputs();
        }
    }
}

// Returns a boolean value if the motor is running
bool StepperMotor::isRunning() const {
    return running_;
}

void StepperMotor::setMicroStepping(const int8_t microStepping) {

    stepsPerRevolution_ = fullSteps * microStepping;

    switch (microStepping) {
        case 2:
            digitalWrite(ms1Pin_, HIGH);
            digitalWrite(ms2Pin_, LOW);
            break;
        case 4:
            digitalWrite(ms1Pin_, LOW);
            digitalWrite(ms2Pin_, HIGH);
            break;
        case 8:
            digitalWrite(ms1Pin_, LOW);
            digitalWrite(ms2Pin_, LOW);
            break;
        case 16:
            digitalWrite(ms1Pin_, HIGH);
            digitalWrite(ms2Pin_, HIGH);
            break;
        default: ;
    }
}

int StepperMotor::getStepsPerRevolution() const {
    return stepsPerRevolution_;
}