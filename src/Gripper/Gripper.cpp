//
// Created by Herman Hårstad Gran on 23/03/2026.
//
#include "Gripper/Gripper.hpp"
#include "Actuators/StepperMotor.hpp"
#include "Sensors/LimitSwitch.hpp"
#include "esp_task_wdt.h"
#include <Debug/Logger.hpp>

Gripper::Gripper()
    : stepper_(5, 12, 11, current_),
      limit_(4),
      current_(2)
{}

void Gripper::init() {
    stepper_.init();
    limit_.init();
    current_.init();
}


void Gripper::homing() {

    stepper_.getAccelStepper()->enableOutputs();
    auto s = stepper_.getAccelStepper();
    getStepper().setSpeed(4000);
    getStepper().setAcceleration(2000);
    s->setSpeed(-2000);

    logger.logf("Waiting for limit switch...");
    while (!limit_.isPressed()) {
        s->runSpeed();
    }

    logger.logf("Limit switch pressed!");

    s->setSpeed(400);
    while (limit_.isPressed()) {

        s->runSpeed();
    }

    stepper_.setHomePos();
    stepper_.getAccelStepper()->disableOutputs();
    gripperState_ = GripperState::HOME;
}

// Large help from claude code and claude
// https://www.youtube.com/watch?v=p4sDgQ-jao4 Function pointers
// https://www.youtube.com/watch?v=PcAD6gjNUVw Function pointers as state machine
// https://www.beningo.com/158-state-machines-with-function-pointers/
void Gripper::stepperTaskWrapper(void* param) {
    auto* g = static_cast<Gripper*>(param);
    esp_task_wdt_add(nullptr);  // this task feeds the WDT while the idle task cannot

    while (g->tasksRunning_) {

        // State machine loop funciton
        g->Sm_Loop();

        esp_task_wdt_reset();
        taskYIELD();
    }

    esp_task_wdt_delete(nullptr);
    xTaskNotifyGive(g->callerTaskHandle_);
    vTaskSuspend(nullptr);
}

// Created with claude code. Prints the current measurements to the UDP logger.
void Gripper::sensorTaskWrapper(void* param) {
    auto* g = static_cast<Gripper*>(param);

    while (g->tasksRunning_) {
        g->getCurrentSensor().updateReading();
        g->getCurrentSensor().printTelemetry();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    // Notify the caller so it knows this task has finished and is safe to delete
    xTaskNotifyGive(g->callerTaskHandle_);
    vTaskSuspend(nullptr);
}


// Created by claude code. To be able to run the stepper motor while also logging
// current values without sowing the motor down.
void Gripper::moveToPosition(const int position) {
    if (stepperTaskHandle != nullptr) return;

    auto* g = getStepper().getAccelStepper();

    getStepper().stop();
    g->setCurrentPosition(g->currentPosition());

    stepper_.runToPosition(position);
    tasksRunning_     = true;
    callerTaskHandle_ = xTaskGetCurrentTaskHandle();

    // The stepper task on core 0 never yields long enough for the core-0 idle task
    // to run, so the idle task can't feed the TWDT. Remove it from monitoring for
    // the duration of the move; the stepper task feeds the TWDT itself instead.
    TaskHandle_t idle0 = xTaskGetIdleTaskHandleForCPU(0);
    esp_task_wdt_delete(idle0);

    xTaskCreatePinnedToCore(stepperTaskWrapper, "StepperTask", 8192, this, 2, &stepperTaskHandle, 0);
    xTaskCreatePinnedToCore(sensorTaskWrapper,  "SensorTask",  8192, this, 1, &sensorTaskHandle,  1);

    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);  // stepper done
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);  // sensor done — safe to delete now

    vTaskDelete(stepperTaskHandle);
    vTaskDelete(sensorTaskHandle);
    stepperTaskHandle = nullptr;
    sensorTaskHandle  = nullptr;

    esp_task_wdt_add(idle0);  // restore idle task WDT monitoring
}

void Gripper::home() {
    gripperState_ = GripperState::RELEASING;
    getStepper().setSpeed(4000);
    getStepper().setAcceleration(2000);
    moveToPosition(homePos_);
}

void Gripper::idlePos() {
    gripperState_ = GripperState::RELEASING;
    getStepper().setSpeed(4000);
    getStepper().setAcceleration(2000);
    moveToPosition(idlePos_);
}

bool Gripper::latch() {
    gripperState_ = GripperState::LATCHING;
    getStepper().setSpeed(3000);
    getStepper().setAcceleration(2000);
    moveToPosition(fullyExtended_);

    switch (gripperState_) {
        case GripperState::FAILED:
            logger.logf("Failed to grip");
            return false;
        case GripperState::OBSTACLE_DETECTED:
            logger.logf("Obstacle detected");
            return false;
        case GripperState::LATCHED:
            logger.logf("Latched");
            return true;
        default:
            return false;
    }
}

StepperMotor& Gripper::getStepper() {
    return stepper_;
}

CurrentSensor& Gripper::getCurrentSensor() {
    return current_;
}

LimitSwitch& Gripper::getLimitSwitch() {
    return limit_;
}

// State machine loop, copied from
// https://www.beningo.com/158-state-machines-with-function-pointers/
void Gripper::Sm_Loop() {

    if (gripperState_ < GripperState::NUM_STATES) {

        (this->*stateMachine_[static_cast<uint8_t>(gripperState_)].handler)();

    } else {
        logger.logf("Invalid gripper state: %d", gripperState_);
        gripperState_ = GripperState::FAILED;
    }

}

// State machine Homeing state
void Gripper::Sm_Home() {

}

// State machine Latching state
void Gripper::Sm_Latching() {
    getStepper().run();

    const int currentPos = getStepper().getAccelStepper()->currentPosition();
    const bool contact = getCurrentSensor().isLatched(currentThreshold_);
    const bool inLatchZone = currentPos > latchZoneStart_;

    if (contact) {
        if (contactStartMs_ == 0) contactStartMs_ = millis();

        if (millis() - contactStartMs_ >= contactDebounceMs_) {
            contactStartMs_ = 0;
            if (!inLatchZone) {
                getStepper().stop();
                getStepper().getAccelStepper()->moveTo(currentPos);
                gripperState_ = GripperState::OBSTACLE_DETECTED;
            } else {
                logger.logf("Contact in latch zone at pos %d — tightening", currentPos);
                getStepper().setSpeed(tightenSpeed_);
                getStepper().setAcceleration(500);
                getStepper().getAccelStepper()->moveTo(currentPos + tightenSteps_);
                gripperState_ = GripperState::TIGHTENING;
            }
        }
    } else {
        contactStartMs_ = 0;
        if (!getStepper().isRunning()) {
            gripperState_ = GripperState::FAILED;
        }
    }
}

// State machine Releasing state
void Gripper::Sm_Releasing() {
    getStepper().run();

    if (!getStepper().isRunning()) {
        gripperState_ = GripperState::IDLE;
    }
}

// State machine Obstacle detected state
void Gripper::Sm_ObstacleDetected() {
    tasksRunning_ = false;
}

// State machine tightening state
void Gripper::Sm_Tighten() {
    getStepper().run();

    if (!getStepper().isRunning()) {
        // Done tightening
        gripperState_ = GripperState::LATCHED;
    }
}

// State machine idle state
void Gripper::Sm_Idle() {
    logger.logf("Moved to pos");
    tasksRunning_ = false;
}

// State machine latched state
void Gripper::Sm_Latched() {
    tasksRunning_ = false;
}

// State machine Failed State
void Gripper::Sm_Failed() {
    tasksRunning_ = false;
}