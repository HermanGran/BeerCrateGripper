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
    logger.logf("Homing...");
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
    logger.logf("Homing done!");
}

static void stepperTaskWrapper(void* param) {
    auto* g = static_cast<Gripper*>(param);
    esp_task_wdt_add(nullptr);  // this task feeds the WDT while the idle task cannot

    while (g->tasksRunning_) {
        g->getStepper().run();
        if (!g->getStepper().isRunning()) {
            g->tasksRunning_ = false;
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

void Gripper::close() {
    moveToPosition(0);
    logger.logf("Gripper closed");
}

void Gripper::open() {
    moveToPosition(3200 * 7);
    logger.logf("Gripper opened");
}

StepperMotor& Gripper::getStepper() {
    return stepper_;
}

CurrentSensor& Gripper::getCurrentSensor() {
    return current_;
}
