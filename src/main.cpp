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
#include <Debug/UDPLogger.hpp>

Gripper gripper;
SerialCommandHandler cmdHandler(gripper);

// --- WiFi config ---
#define WIFI_SSID     "TP-Link_9538"
#define WIFI_PASSWORD "89546543"
#define AGENT_IP      "192.168.0.100"  // Your PC's IP
#define AGENT_PORT    8888

UDPLogger logger;

// Static IP config — change to match your PC's subnet
IPAddress local_ip(192, 168, 0, 102);   // ESP32's desired IP
IPAddress gateway(192, 168, 0, 1);       // Your router's IP
IPAddress subnet(255, 255, 255, 0);

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

void destroy_entities() {
    rclc_executor_fini(&executor);
    rcl_publisher_fini(&publisher, &node);
    rcl_timer_fini(&timer);
    rclc_support_fini(&support);
    rcl_node_fini(&node);
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        msg.data++;
        rcl_publish(&publisher, &msg, NULL);
        logger.logf("Published: %d", msg.data);
    }
}

void setup() {
    Serial.begin(115200);
    delay(3000);

    //WiFi.config(local_ip, gateway, subnet);  // static IP — keeps OTA port stable
    set_microros_wifi_transports(
        WIFI_SSID,
        WIFI_PASSWORD,
        IPAddress(192, 168, 0, 100),
        AGENT_PORT
    );

    // UDP socket must be opened after WiFi connects
    logger.init(IPAddress(192, 168, 0, 104), 4444);

    // OTA setup
    ArduinoOTA.setHostname("gripper-esp32");  // Optional friendly name
    ArduinoOTA.setPassword("herman");  // Optional password

    logger.logf("Logger started");

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

    gripper.init();
    cmdHandler.init();

}

unsigned long lastMillis = 0;
int i = 0;

void loop() {
    ArduinoOTA.handle();  // Must be called regularly — add at top of loop

    // Check WiFi is still connected
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi lost, reconnecting...");
        WiFi.reconnect();
        delay(1000);
        return;
    }
    /*
    switch (agent_state) {
        case WAITING_AGENT:
            Serial.println("Waiting for agent...");
            if (RMW_RET_OK == rmw_uros_ping_agent(1000, 3)) {  // 1000ms timeout, 3 attempts
                agent_state = AGENT_AVAILABLE;
            }
            break;

        case AGENT_AVAILABLE:
            if (create_entities()) {
                Serial.println("Agent connected!");
                agent_state = AGENT_CONNECTED;
            } else {
                destroy_entities();
                agent_state = WAITING_AGENT;
            }
            break;

        case AGENT_CONNECTED:
            // More lenient ping — 500ms timeout, 2 attempts
            if (RMW_RET_OK == rmw_uros_ping_agent(500, 2)) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            } else {
                Serial.println("Agent lost, reconnecting...");
                agent_state = AGENT_DISCONNECTED;
            }
            break;

        case AGENT_DISCONNECTED:
            destroy_entities();
            agent_state = WAITING_AGENT;
            break;
    }
    */
}
