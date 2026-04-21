//
// Created by Herman Hårstad Gran on 23/03/2026.
//
#include "Gripper/Gripper.hpp"
#include "Actuators/StepperMotor.hpp"
#include "Sensors/LimitSwitch.hpp"
#include "esp_task_wdt.h"

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
    esp_task_wdt_add(NULL);  // this task feeds the WDT while the idle task cannot

    while (g->tasksRunning_) {
        g->getStepper().run();
        if (!g->getStepper().isRunning()) {
            g->tasksRunning_ = false;
        }
        esp_task_wdt_reset();
        taskYIELD();
    }

    esp_task_wdt_delete(NULL);
    xTaskNotifyGive(g->callerTaskHandle_);
    vTaskSuspend(NULL);
}

static void sensorTaskWrapper(void* param) {
    Gripper* g = static_cast<Gripper*>(param);

    while (g->tasksRunning_) {
        g->getCurrentSensor().printTelemetry();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Notify caller so it knows this task has finished and is safe to delete
    xTaskNotifyGive(g->callerTaskHandle_);
    vTaskSuspend(NULL);
}

void Gripper::moveToPosition(int position) {
    if (stepperTaskHandle != NULL) return;

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
    stepperTaskHandle = NULL;
    sensorTaskHandle  = NULL;

    esp_task_wdt_add(idle0);  // restore idle task WDT monitoring
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
