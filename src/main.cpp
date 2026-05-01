#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <Gripper/Gripper.hpp>
#include <Debug/Logger.hpp>
#include <ROS/MicroRosConnection.hpp>
#include <ROS/GripperNode.hpp>

Gripper gripper;
GripperNode gripperNode(gripper);

UDPLogger logger;
MicroRosConnection mrCon;

void setup() {
    Serial.begin(115200);

    mrCon.initWiFi();
    logger.init(IPAddress(192, 168, 0, 102), 4444);
    logger.logf("Logger started");
    mrCon.initOTA("esp32-gripper", "herman");

    gripper.init();
    mrCon.registerNode(&gripperNode);
}

void loop() {
    mrCon.updateOTA();
    mrCon.update();
}
