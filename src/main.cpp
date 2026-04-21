#include <Arduino.h>
#include <Gripper/Gripper.hpp>
#include <Gripper/SerialCommandHandler.hpp>

Gripper gripper;
SerialCommandHandler cmdHandler(gripper);

void setup() {
    gripper.init();
    cmdHandler.init();

}

void loop() {
    cmdHandler.update();
}