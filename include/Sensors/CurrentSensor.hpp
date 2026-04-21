//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#ifndef BEERCRATEGRIPPER_CURRENTSENSOR_HPP
#define BEERCRATEGRIPPER_CURRENTSENSOR_HPP

#include <Arduino.h>

class CurrentSensor {
public:
    // pin        : analog pin connected to ACS723 OUT
    // vcc        : supply voltage (3.3f for ESP32)
    // sensitivity: V/A — 0.4f for ACS723 ±5 A, 0.2f for ±10 A
    // samples    : number of ADC readings to average per measurement
    CurrentSensor(int pin, float vcc = 3.3f, float sensitivity = 0.4f, int samples = 50);

    void  init();

    // Returns averaged current in amps (signed — negative means reverse flow)
    float readCurrentA();

    // Returns true when |current| exceeds thresholdA — use for latch detection
    bool  isLatched(float thresholdA);

    // Prints "millis,amps\n" to Serial — call in a loop for Python live-plotting
    void  printTelemetry();

private:
    int   pin_;
    float vcc_;
    float sensitivity_;
    int   samples_;
    float zeroOffset_;   // measured quiescent voltage in V, set by begin()

    float sampleAvgV();  // average ADC reading converted to volts
};

#endif //BEERCRATEGRIPPER_CURRENTSENSOR_HPP
