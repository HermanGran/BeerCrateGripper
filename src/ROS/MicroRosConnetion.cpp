//
// Created by Herman Hårstad Gran on 23/04/2026.
//

#include "Arduino.h"
#include <ArduinoOTA.h>
#include <micro_ros_platformio.h>
#include <rclc/node.h>

#include <Debug/Logger.hpp>
#include <ROS/MicroRosConnection.hpp>

void MicroRosConnection::registerNode(RosNode *node) {
    nodes.push_back(node);
}


void MicroRosConnection::initWiFi() {
    delay(3000);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, LOW);

    IPAddress localIP(192, 168, 0, 104);
    IPAddress gateway(192, 168, 0, 1);
    IPAddress subnet(255, 255, 255, 0);

    WiFi.config(localIP, gateway, subnet);

    set_microros_wifi_transports(
        WIFI_SSID,
        WIFI_PASSWORD,
        IPAddress(192, 168, 0, 100),
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

void MicroRosConnection::updateOTA() {
    ArduinoOTA.handle();

    if (WiFi.status() != WL_CONNECTED) {
        logger.logf("WiFi lost, reconnecting...");
        WiFi.reconnect();
        digitalWrite(LED_RED, LOW);
        delay(1000);
        return;
    }
    digitalWrite(LED_RED, HIGH);

}


void MicroRosConnection::update() {

    switch (state) {
        case WAITING_AGENT:
            logger.logf("Waiting for agent...");
            if (RMW_RET_OK == rmw_uros_ping_agent(1000, 3)) {
                logger.logf("Ping OK! Creating entities...");
                state = AGENT_AVAILABLE;
            }
            break;

        case AGENT_AVAILABLE:
            delay(500);  // let agent finish tearing down the ping session
            if (createEntities()) {
                logger.logf("Agent connected!");
                state = AGENT_CONNECTED;
            } else {
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
            set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, IPAddress(192, 168, 0, 100), AGENT_PORT);
            state = WAITING_AGENT;
            break;
    }
}

bool MicroRosConnection::createEntities() {
    allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
        logger.logf("FAIL: rcl_init_options_init"); return false;
    }
    if (rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID) != RCL_RET_OK) {
        logger.logf("FAIL: set_domain_id");
        rcl_init_options_fini(&init_options);
        return false;
    }
    if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
        logger.logf("FAIL: support_init");
        rcl_init_options_fini(&init_options);
        return false;
    }
    rcl_init_options_fini(&init_options);

    if (rclc_node_init_default(&node, "gripper_node", "", &support) != RCL_RET_OK) {
        logger.logf("FAIL: node_init");
        rclc_support_fini(&support);
        return false;
    }

    int total_handles = 0;
    for (auto* n : nodes) total_handles += n->executor_handles();
    logger.logf("Executor handles: %d", total_handles);

    if (rclc_executor_init(&executor, &support.context, total_handles, &allocator) != RCL_RET_OK) {
        logger.logf("FAIL: executor_init");
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }

    for (auto* n : nodes) {
        if (!n->init(&node, &support, &executor)) {
            logger.logf("FAIL: node->init()");
            rclc_executor_fini(&executor);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            return false;
        }
    }

    logger.logf("All entities created OK");
    return true;
}

void MicroRosConnection::destroyEntities() {
    for (auto* n : nodes) n->fini(&node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}