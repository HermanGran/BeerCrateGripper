//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#include "Sensors/CurrentSensor.hpp"
#include <Arduino.h>
#include <Debug/Logger.hpp>

CurrentSensor::CurrentSensor(int pin, float vcc, float sensitivity, int samples)
    : pin_(pin), vcc_(vcc), sensitivity_(sensitivity), samples_(samples), zeroOffset_(vcc / 2.0f)
{}

void CurrentSensor::init() {
    analogReadResolution(12);   // ESP32: use full 12-bit range (0–4095)
    pinMode(pin_, INPUT);

    // Calibrate zero-current offset: motor must be stopped during begin()
    zeroOffset_ = sampleAvgV();
}

float CurrentSensor::sampleAvgV() {
    long sum = 0;
    for (int i = 0; i < samples_; i++) {
        sum += analogRead(pin_);
    }
    float avgRaw = static_cast<float>(sum) / samples_;
    return (avgRaw / 4095.0f) * vcc_;
}

float CurrentSensor::readCurrentA() {
    float voltage = sampleAvgV();
    return (voltage - zeroOffset_) / sensitivity_;
}

bool CurrentSensor::isLatched(float thresholdA) {
    return fabsf(readCurrentA()) >= thresholdA;
}

void CurrentSensor::printTelemetry() {
    // Format: "millis,amps\n"  — easy to parse with Python serial + matplotlib
    logger.logf("Current: %.4f A", readCurrentA());
}
