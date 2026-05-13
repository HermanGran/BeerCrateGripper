//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#include "Sensors/CurrentSensor.hpp"
#include <Arduino.h>
#include <Debug/Logger.hpp>

CurrentSensor::CurrentSensor(const int pin, const float vcc, const float sensitivity, const int samples)
    : pin_(pin), vcc_(vcc), sensitivity_(sensitivity), samples_(samples), zeroOffset_(vcc / 2.0f)
{}

void CurrentSensor::init() {
    analogReadResolution(12);   // full 12-bit range (0–4095)
    pinMode(pin_, INPUT);

    // Calibrate zero-current offset
    zeroOffset_ = sampleAvgV();
}

float CurrentSensor::sampleAvgV() const {
    long sum = 0;
    for (int i = 0; i < samples_; i++) {
        sum += analogRead(pin_);
        delayMicroseconds(100);  // spread samples across PWM switching cycles
    }
    const float avgRaw = static_cast<float>(sum) / samples_;
    return (avgRaw / 4095.0f) * vcc_;
}

float CurrentSensor::readCurrentA() const {
    const float voltage = sampleAvgV();
    return (voltage - zeroOffset_) / sensitivity_;
}

bool CurrentSensor::isLatched(const float thresholdA) const {
    return fabsf(getLatestReading()) >= thresholdA;
}

void CurrentSensor::printTelemetry() const {
    // Format: "millis,amps\n"  — easy to parse with Python serial + matplotlib
    logger.logf("Current: %.4f A", getLatestReading());
}

// Help from claude to crete this function
void CurrentSensor::updateReading() {
    const float prev = currentA_.load(std::memory_order_relaxed);
    constexpr float alpha = 0.3f;
    const float newReading = alpha * readCurrentA() + (1 - alpha) * prev;
    currentA_.store(newReading, std::memory_order_relaxed);
}

// Help from claude to create this function returns latest reading updated by the updateReading function
float CurrentSensor::getLatestReading() const {
    return currentA_.load(std::memory_order_relaxed);
}
