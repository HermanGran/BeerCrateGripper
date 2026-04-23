//
// Created by Herman Hårstad Gran on 23/04/2026.
//

#include "Arduino.h"
#include <ArduinoOTA.h>
#include <micro_ros_platformio.h>

#include <Debug/Logger.hpp>
#include <ROS/MicroRosConnection.hpp>

void MicroRosConnection::registerNode(RosNode *node) {
    nodes.push_back(node);
}


void MicroRosConnection::initWiFi() {
    delay(3000);
    set_microros_wifi_transports(
        WIFI_SSID,
        WIFI_PASSWORD,
        agent_ip,
        AGENT_PORT
    );

}

void MicroRosConnection::initOTA(const char *hostName, const char *password) {
    // OTA setup
    ArduinoOTA.setHostname(hostName);
    ArduinoOTA.setPassword(password);

    ArduinoOTA.onStart([]() {
        logger.logf("OTA Start");
    });

    ArduinoOTA.onEnd([]() {
        logger.logf("OTA Done!");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        logger.logf("Progress: %u%%\n", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        logger.logf("OTA Error %u\n", error);
    });

    ArduinoOTA.begin();

    logger.logf("OTA Ready!");
    logger.logf("ESP32 connected! IP: %s", WiFi.localIP().toString().c_str());
}

void MicroRosConnection::update() {
    ArduinoOTA.handle();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi lost, reconnecting...");
        WiFi.reconnect();
        delay(1000);
        return;
    }

    switch (state) {
        case WAITING_AGENT:
            logger.logf("Waiting for agent...");
            if (RMW_RET_OK == rmw_uros_ping_agent(1000, 3)) {  // 1000ms timeout, 3 attempts
                state = AGENT_AVAILABLE;
            }
            break;

        case AGENT_AVAILABLE:
            if (createEntities()) {
                logger.logf("Agent connected!");
                state = AGENT_CONNECTED;
            } else {
                destroyEntities();
                state = WAITING_AGENT;
            }
            break;

        case AGENT_CONNECTED:
            // More lenient ping — 500ms timeout, 2 attempts
            if (RMW_RET_OK == rmw_uros_ping_agent(500, 2)) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            } else {
                logger.logf("Agent lost, reconnecting...");
                state = AGENT_DISCONNECTED;
            }
            break;

        case AGENT_DISCONNECTED:
            destroyEntities();
            state = WAITING_AGENT;
            break;
    }
}

bool MicroRosConnection::createEntities() {
    allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&node, "gripper_node", "", &support));


    // Count total executor handles from all registered nodes
    int total_handles = 0;
    for (auto* n : nodes) total_handles += n->executor_handles();
    RCCHECK(rclc_executor_init(&executor, &support.context, total_handles, &allocator));

    // Init all registered nodes
    for (auto* n : nodes) {
        if (!n->init(&node, &executor)) return false;
    }

    return true;
}

void MicroRosConnection::destroyEntities() {
    for (auto* n : nodes) n->fini(&node);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
    rcl_node_fini(&node);
}