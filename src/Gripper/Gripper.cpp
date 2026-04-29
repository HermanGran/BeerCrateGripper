//
// Created by Herman Hårstad Gran on 23/03/2026.
//
#include "Gripper/Gripper.hpp"
#include "Actuators/StepperMotor.hpp"
#include "Sensors/LimitSwitch.hpp"
#include "esp_task_wdt.h"
#include <Debug/Logger.hpp>

TaskHandle_t stepperTaskHandle = nullptr;
TaskHandle_t sensorTaskHandle  = nullptr;

#define STEPS 3200 // Number of steps in one rotation
#define LATCH_ZONE_START  (STEPS * 5)    // Steps — after this we expect to hit something
#define TIGHTEN_STEPS     (STEPS * 0.3)  // Extra steps to tighten after contact
#define CURRENT_THRESHOLD  0.400f       // Amps — contact detected above this
#define TIGHTEN_SPEED      400          // Slower speed for tightening

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
    gripperState_ = MOVING;

    stepper_.getStepper()->enableOutputs();
    auto s = stepper_.getStepper();
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
    stepper_.getStepper()->disableOutputs();
    gripperState_ = HOME;
}

// Created with claude code
static void stepperTaskWrapper(void* param) {
    auto* g = static_cast<Gripper*>(param);
    esp_task_wdt_add(nullptr);  // this task feeds the WDT while the idle task cannot

    int tightenStepsRemaining = TIGHTEN_STEPS;

    while (g->tasksRunning_) {

        const int currentPos = g->getStepper().getStepper()->currentPosition();
        const bool contact = g->getCurrentSensor().isLatched(CURRENT_THRESHOLD);
        const bool inLatchZone = currentPos > LATCH_ZONE_START;

        switch (g->getGripperState()) {
            case Gripper::GripperState::MOVING:
                g->getStepper().run();
                if (contact && !inLatchZone) {
                    // Hit something too early, stopping
                    g->getStepper().stop();
                    logger.logf("Obstacle detected at pos %d — stopping!", currentPos);

                    g->setGripperState(Gripper::GripperState::OBSTACLE_DETECTED);
                    g->tasksRunning_ = false;

                } else if (contact && inLatchZone) {
                    // Hit something in latch zone — slow down and tighten
                    logger.logf("Contact in latch zone at pos %d — tightening", currentPos);
                    g->getStepper().setSpeed(TIGHTEN_SPEED);
                    g->getStepper().setAcceleration(500);
                    g->getStepper().getStepper()->move(TIGHTEN_STEPS);
                    g->setGripperState(Gripper::GripperState::TIGHTENING);

                } else if (!g->getStepper().isRunning()) {
                    // Reached end without contact
                    logger.logf("Reached end without contact — failed to grip");
                    g->setGripperState(Gripper::GripperState::FAILED);
                    g->tasksRunning_ = false;
                }
                break;

            case Gripper::GripperState::TIGHTENING:
                g->getStepper().run();
                tightenStepsRemaining--;

                if (!g->getStepper().isRunning() || tightenStepsRemaining <= 0) {
                    // Done tightening
                    logger.logf("Tightening done — latched!");
                    g->getStepper().stop();
                    g->setGripperState(Gripper::GripperState::LATCHED);
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

static void sensorTaskWrapper(void* param) {
    auto* g = static_cast<Gripper*>(param);

    while (g->tasksRunning_) {
        g->getCurrentSensor().printTelemetry();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    // Notify caller so it knows this task has finished and is safe to delete
    xTaskNotifyGive(g->callerTaskHandle_);
    vTaskSuspend(nullptr);
}

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
    moveToPosition(0);
}

bool Gripper::latch() {
    setGripperState(MOVING);
    moveToPosition(3200 * 7.1);

    switch (gripperState_) {
        case FAILED:
            logger.logf("Failed to grip");
            return false;
        case OBSTACLE_DETECTED:
            logger.logf("Obstacle detected");
            return false;
        case LATCHED:
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

Gripper::GripperState Gripper::getGripperState() const {
    return gripperState_;
}

void Gripper::setGripperState(const GripperState state) {
    gripperState_ = state;
}