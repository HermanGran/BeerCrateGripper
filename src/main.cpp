#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include "WiFi.h"
#include <ArduinoOTA.h>

#include <Gripper/Gripper.hpp>
#include <Gripper/SerialCommandHandler.hpp>
#include <Debug/Logger.hpp>
#include <ROS/MicroRosConnection.hpp>

Gripper gripper;
SerialCommandHandler cmdHandler(gripper);

// --- WiFi config ---
#define WIFI_SSID     "TP-Link_9538"
#define WIFI_PASSWORD "89546543"
#define AGENT_IP      "192.168.0.100"  // Your PC's IP
#define AGENT_PORT    8888

UDPLogger logger;

// --- micro-ROS objects ---
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_publisher_t publisher;
rcl_timer_t timer;
std_msgs__msg__Int32 msg;

// --- State machine ---
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agent_state = WAITING_AGENT;

// --- Forward declaration ---
void timer_callback(rcl_timer_t* timer, int64_t last_call_time);

#define RCCHECK(fn) { if(fn != RCL_RET_OK) { return false; }}

bool create_entities() {
    allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 7));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_counter_node", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "esp32/counter"
    ));

    RCCHECK(rclc_timer_init_default(
        &timer, &support,
        RCL_MS_TO_NS(1000),
        timer_callback
    ));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg.data = 0;
    return true;
}



void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        msg.data++;
        rcl_publish(&publisher, &msg, NULL);
        logger.logf("Published: %d", msg.data);
    }
}

MicroRosConnection mrCon;

void setup() {
    Serial.begin(115200);

    mrCon.initWiFi();
    logger.init(IPAddress(192, 168, 0, 104), 4444);
    logger.logf("Logger started");
    mrCon.initOTA("esp32-gripper", "herman");

    gripper.init();
    cmdHandler.init();

}

void loop() {
    mrCon.update();

}
