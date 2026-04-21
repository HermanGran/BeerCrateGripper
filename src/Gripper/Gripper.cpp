//
// Created by Herman Hårstad Gran on 23/03/2026.
//
#include "Gripper/Gripper.hpp"
#include "Actuators/StepperMotor.hpp"
#include "Sensors/LimitSwitch.hpp"

TaskHandle_t stepperTaskHandle = NULL;
TaskHandle_t sensorTaskHandle  = NULL;

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
    Serial.println("Homing...");
    auto s = stepper_.getStepper();
    s->setSpeed(-2000);


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
    Serial.println("Homing done!");
}

static void stepperTaskWrapper(void* param) {
    Gripper* g = static_cast<Gripper*>(param);

    while (g->getStepper().isRunning()) {
        g->getStepper().run();
        taskYIELD();
    }

    // Notify caller that stepper is done — don't self-delete
    xTaskNotifyGive(g->callerTaskHandle_);
    vTaskSuspend(NULL);  // Suspend and wait to be deleted by caller
}

static void sensorTaskWrapper(void* param) {
    Gripper* g = static_cast<Gripper*>(param);

    while (g->getStepper().isRunning()) {
        g->getCurrentSensor().printTelemetry();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskSuspend(NULL);  // Suspend and wait to be deleted by caller
}

void Gripper::moveToPosition(int position) {
    if (stepperTaskHandle != NULL) return;  // Already running, ignore
    stepper_.runToPosition(position);
    callerTaskHandle_ = xTaskGetCurrentTaskHandle();  // Save caller handle

    xTaskCreatePinnedToCore(
        stepperTaskWrapper,
        "StepperTask",
        4000,
        this,
        2,
        &stepperTaskHandle,
        0
        );

    xTaskCreatePinnedToCore(
        sensorTaskWrapper,
        "SensorTask",
        4000,
        this,
        1,
        &sensorTaskHandle,
        1
        );

    // Block until stepper task signals completion
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Motor is done — now safe to delete both tasks
    vTaskDelay(20 / portTICK_PERIOD_MS);  // Small grace period for sensor task to suspend
    vTaskDelete(stepperTaskHandle);
    vTaskDelete(sensorTaskHandle);
    stepperTaskHandle = NULL;
    sensorTaskHandle  = NULL;
}

void Gripper::close() {
    moveToPosition(0);
    Serial.println("Closed");
}

void Gripper::open() {
    moveToPosition(3200 * 6);
    Serial.println("Opened");
}

StepperMotor& Gripper::getStepper() {
    return stepper_;
}

CurrentSensor& Gripper::getCurrentSensor() {
    return current_;
}
