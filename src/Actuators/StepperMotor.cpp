//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#include "Actuators/StepperMotor.hpp"

#include "Debug/Logger.hpp"

// Constructor takes in pins for the motor driver and the current sensor
StepperMotor::StepperMotor(const int EN_PIN, const int DIR_PIN, const int STEP_PIN)
    :   enPin_(EN_PIN),
        dirPin_(DIR_PIN),
        stepPin_(STEP_PIN)
{}

// Initialization function, initializes the pins for the motor driver and sets the speeds and acceleration of the motor.
void StepperMotor::init() {
    engine_ = new FastAccelStepperEngine();
    engine_->init();
    stepper_ = engine_->stepperConnectToPin(stepPin_, DRIVER_MCPWM_PCNT);

    pinMode(enPin_, OUTPUT);

    if (stepper_ == nullptr) {
        logger.logf("Failed to create stepper — pin may not support hardware timer");
        while (true) {}
    }
    logger.logf("Stepper connected OK on pin %d", stepPin_);

    stepper_->setDirectionPin(dirPin_);
    stepper_->setEnablePin(enPin_, false);  // TMC2208: EN active LOW
    stepper_->setAutoEnable(true);
    setSpeedInHz(4000);
    setAcceleration(2000);

}

// Sets the speed of the stepper motor
void StepperMotor::setSpeedInHz(const int speed) const {
    stepper_->setSpeedInHz(speed);
}

// Sets the acceleration of the motor
void StepperMotor::setAcceleration(const int acceleration) const {
    stepper_->setAcceleration(acceleration);
}

// Sets the current position of the motor as the new home
void StepperMotor::setHomePos() const {
    stepper_->setCurrentPosition(0);
}

// Run forward
void StepperMotor::runForward() const {
    stepper_->runForward();
}

// Run backwards
void StepperMotor::runBackward() const {
    stepper_->runBackward();
}

// Immediate hard stop
void StepperMotor::forceStop() const {
    stepper_->forceStop();
}

// Decelerates smoothly to a stop
void StepperMotor::stopMove() const {
    stepper_->stopMove();
}



// Runs to a position in number of steps, the number of steps is stored internally
void StepperMotor::moveTo(const int position) const {
    const MoveResultCode err = stepper_->moveTo(position);
    if (err != MoveResultCode::OK) {
        logger.logf("FastAccelStepper moveTo(%d) error: %d\n", position, err);
    }
}


// Returns a boolean value if the motor is running
bool StepperMotor::isRunning() const {
    return stepper_->isRunning();
}

// Returns the current position of the motor
int StepperMotor::getPosition() const {
    return stepper_->getCurrentPosition();
}
