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
    gripperState_ = GripperState::MOVING;

    stepper_.getAccelStepper()->enableOutputs();
    auto s = stepper_.getAccelStepper();
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
void Gripper::stepperTaskWrapper(void* param) {
    auto* g = static_cast<Gripper*>(param);
    esp_task_wdt_add(nullptr);  // this task feeds the WDT while the idle task cannot

    while (g->tasksRunning_) {

        const int currentPos = g->getStepper().getAccelStepper()->currentPosition();
        const bool contact = g->getCurrentSensor().isLatched(currentThreshold_);
        const bool inLatchZone = currentPos > latchZoneStart_;

        switch (g->gripperAction_) {
            case GripperAction::LATCH:

                switch (g->gripperState_) {
                    case GripperState::MOVING:
                        g->getStepper().run();
                        if (contact && !inLatchZone) {
                            // Hit something too early, stopping
                            g->getStepper().stop();
                            g->getStepper().getAccelStepper()->moveTo(currentPos);
                            logger.logf("Obstacle detected at pos %d — stopping!", currentPos);

                            g->gripperState_ = GripperState::OBSTACLE_DETECTED;
                            g->tasksRunning_ = false;

                        } else if (contact && inLatchZone) {
                            // Hit something in latch zone — slow down and tighten
                            logger.logf("Contact in latch zone at pos %d — tightening", currentPos);
                            g->getStepper().setSpeed(tightenSpeed_);
                            g->getStepper().setAcceleration(500);
                            g->getStepper().getAccelStepper()->moveTo(currentPos + tightenSteps_);
                            g->gripperState_ = GripperState::TIGHTENING;

                        } else if (!g->getStepper().isRunning()) {
                            // Reached end without contact
                            logger.logf("Reached end without contact — failed to grip");
                            g->gripperState_ = GripperState::FAILED;
                            g->tasksRunning_ = false;
                        }
                        break;

                    case GripperState::TIGHTENING:
                        g->getStepper().run();

                        if (!g->getStepper().isRunning()) {
                            // Done tightening
                            logger.logf("Tightening done — latched!");
                            g->getStepper().stop();
                            g->gripperState_ = GripperState::LATCHED;
                            g->tasksRunning_ = false;
                        }
                        break;

                    default:
                        g->tasksRunning_ = false;
                        break;
                }
                break;

            case GripperAction::RELEASE:
                g->getStepper().run();

                if (!g->getStepper().isRunning()) {
                    // Done tightening
                    logger.logf("Moved to idle position");
                    g->gripperState_ = GripperState::IDLE;
                    g->tasksRunning_ = false;
                }
                break;
            default:
                g->tasksRunning_ = false;
                break;
        }


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

void Gripper::release() {
    gripperState_ = GripperState::MOVING;
    moveToPosition(idlePos_);
    gripperState_ = GripperState::HOME;
}

bool Gripper::latch() {
    gripperState_ = GripperState::MOVING;
    moveToPosition(fullyExtended_);

    switch (gripperState_) {
        case GripperState::FAILED:
            logger.logf("Failed to grip");
            return false;
        case GripperState::OBSTACLE_DETECTED:
            logger.logf("Obstacle detected");
            return false;
        case GripperState::LATCHED:
            logger.logf("Gripped");
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
