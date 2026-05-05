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


void MicroRosConnection::initWiFi(const char* ssid, const char* password, const IPAddress& ros_agent) {
    ssid_ = ssid;
    password_ = password;
    agent_ip_ = ros_agent;
    delay(1000);

    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, LOW);

    WiFi.config(localIP, gateway, subnet);

    // Tip from claude to const_cast ssid and password to keep const correctness elsewhere
    set_microros_wifi_transports(
        const_cast<char*>(ssid_),
        const_cast<char*>(password_),
        agent_ip_,
        agent_port_
    );

    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);
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

    ArduinoOTA.onProgress([](const unsigned int progress, const unsigned int total) {
        logger.logf("Progress: %u%%\n", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](const ota_error_t error) {
        logger.logf("OTA Error %u\n", error);
    });

    ArduinoOTA.begin();

    logger.logf("OTA Ready!");
    logger.logf("ESP32 connected! IP: %s", WiFi.localIP().toString().c_str());
}

void MicroRosConnection::updateOTA() {
    ArduinoOTA.handle();

    if (WiFiClass::status() != WL_CONNECTED) {
        logger.logf("WiFi lost, reconnecting...");
        digitalWrite(LED_RED, LOW);
        delay(1000);
        return;
    }
    digitalWrite(LED_RED, HIGH);

}


void MicroRosConnection::update() {

    switch (state_) {
        case State::WAITING_AGENT:
            logger.logf("Waiting for agent...");
            if (RMW_RET_OK == rmw_uros_ping_agent(1000, 3)) {
                logger.logf("Ping OK! Creating entities...");
                state_ = State::AGENT_AVAILABLE;
            }
            break;

        case State::AGENT_AVAILABLE:
            delay(500);  // let the agent finish tearing down the ping session
            if (createEntities()) {
                logger.logf("Agent connected!");
                state_ = State::AGENT_CONNECTED;
            } else {
                state_ = State::WAITING_AGENT;
            }
            break;

        case State::AGENT_CONNECTED:
            // More lenient ping — 500ms timeout, 2 attempts
            if (RMW_RET_OK == rmw_uros_ping_agent(500, 2)) {
                rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100));
            } else {
                logger.logf("Agent lost, reconnecting...");
                state_ = State::AGENT_DISCONNECTED;
            }
            break;

        case State::AGENT_DISCONNECTED:
            destroyEntities();
            set_microros_wifi_transports(const_cast<char*>(ssid_), const_cast<char*>(password_), agent_ip_, agent_port_);
            state_ = State::WAITING_AGENT;
            break;
    }
}

bool MicroRosConnection::createEntities() {
    allocator_ = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, allocator_) != RCL_RET_OK) {
        logger.logf("FAIL: rcl_init_options_init"); return false;
    }
    if (rcl_init_options_set_domain_id(&init_options, domain_id_) != RCL_RET_OK) {
        logger.logf("FAIL: set_domain_id");
        const rcl_ret_t rc = rcl_init_options_fini(&init_options);
        if (rc != RCL_RET_OK) {
            logger.logf("rcl_init_options_fini failed: %d", rc);
        }
        return false;
    }
    if (rclc_support_init_with_options(&support_, 0, nullptr, &init_options, &allocator_) != RCL_RET_OK) {
        logger.logf("FAIL: support_init");
        const rcl_ret_t rc = rcl_init_options_fini(&init_options);
        if (rc != RCL_RET_OK) {
            logger.logf("rcl_init_options_fini failed: %d", rc);
        }
        return false;
    }
    {
        const rcl_ret_t rc = rcl_init_options_fini(&init_options);
        if (rc != RCL_RET_OK) {
            logger.logf("rcl_init_options_fini failed: %d", rc);
        }
    }

    node_ = rcl_get_zero_initialized_node();
    if (rclc_node_init_default(&node_, "gripper_node", "", &support_) != RCL_RET_OK) {
        logger.logf("FAIL: node_init");
        rclc_support_fini(&support_);
        return false;
    }

    int total_handles = 0;
    for (auto* n : nodes) total_handles += n->executor_handles();
    logger.logf("Executor handles: %d", total_handles);

    if (rclc_executor_init(&executor_, &support_.context, total_handles, &allocator_) != RCL_RET_OK) {
        logger.logf("FAIL: executor_init");
        const rcl_ret_t rc = rcl_node_fini(&node_);
        if (rc != RCL_RET_OK) {
            logger.logf("rcl_node_fini failed: %d", rc);
        }
        rclc_support_fini(&support_);
        return false;
    }

    for (auto* n : nodes) {
        if (!n->init(&node_, &support_, &executor_)) {
            logger.logf("FAIL: node->init()");
            rclc_executor_fini(&executor_);
            const rcl_ret_t rc = rcl_node_fini(&node_);
            if (rc != RCL_RET_OK) {
                logger.logf("rcl_node_fini failed: %d", rc);
            }
            rclc_support_fini(&support_);
            return false;
        }
    }

    logger.logf("All entities created OK");
    return true;
}

void MicroRosConnection::destroyEntities() {
    // Set destroy timeout to 0 so fini calls don't try to reach the (already gone) agent
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support_.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    for (auto* n : nodes) n->fini(&node_);
    rclc_executor_fini(&executor_);
    rcl_node_fini(&node_);
    rclc_support_fini(&support_);
}