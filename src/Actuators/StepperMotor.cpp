//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#include "Actuators/StepperMotor.hpp"
#include <HardwareSerial.h>
#include <Debug/Logger.hpp>

// Constructor takes in pins for the motor driver and the current sensor
StepperMotor::StepperMotor(const int8_t EN_PIN, const int8_t DIR_PIN, const int8_t STEP_PIN, const int8_t RX_PIN, const int8_t TX_PIN)
    :   stepper_(AccelStepper::DRIVER, STEP_PIN, DIR_PIN),
        enPin_(EN_PIN),
        dirPin_(DIR_PIN),
        stepPin_(STEP_PIN),
        rxPin_(RX_PIN),
        txPin_(TX_PIN)
{}

// Initialization function, initializes the pins for the motor driver and sets the speeds and acceleration of the motor.
void StepperMotor::init() {
    Serial2.begin(115200, SERIAL_8N1, rxPin_, txPin_);

    tmc_ = new TMC2208Stepper(&Serial2, rSense_);

    tmc_->begin();
    tmc_->pdn_disable(true);      // required to enable UART control
    tmc_->mstep_reg_select(true); // use UART for microstepping, not MS pins
    //tmc_->en_spreadCycle(true);   // enable spreadCycle for high speed
    tmc_->rms_current(800);       // set motor current in mA — tune to your motor
    tmc_->microsteps(8);         // set microstepping via UART
    tmc_->ihold(8);               // lower holding current when idle — reduces heat
    tmc_->pwm_autoscale(true);

    delay(500);

    const uint8_t version = tmc_->version();
    if (version == 0x21) {
        logger.logf("TMC2208 UART OK — version: 0x%02X", version);
    } else {
        logger.logf("TMC2208 UART FAILED — got: 0x%02X, CRCerror: %d", version, (int)tmc_->CRCerror);
    }


    pinMode(enPin_, OUTPUT);
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

