#include <Arduino.h>
#include <Gripper/Gripper.hpp>

Gripper gripper;


void setup() {
    Serial.begin(115200);
    gripper.init();
}

void loop() {
    Serial.println("Starting homing");
    gripper.homing();
    Serial.println("Homing done");

    delay(10000);
}