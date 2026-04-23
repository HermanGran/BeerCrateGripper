//
// Created by Herman Hårstad Gran on 23/04/2026.
//

#ifndef BEERCRATEGRIPPER_WIFI_HPP
#define BEERCRATEGRIPPER_WIFI_HPP

#include <vector>

#include <ROS/RosNode.hpp>


#define WIFI_SSID     "TP-Link_9538"
#define WIFI_PASSWORD "89546543"
#define ROS_DOMAIN_ID 7
#define AGENT_PORT    8888

class MicroRosConnection {
public:

    enum State { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };

    rcl_node_t      node;
    rcl_allocator_t allocator;
    rclc_support_t  support;
    rclc_executor_t executor;
    State           state = WAITING_AGENT;

    std::vector<RosNode*> nodes;  // All registered ROS components

    void registerNode(RosNode * node);

    void initWiFi();

    void initOTA(const char* hostName, const char* password);

    void update();

private:
    IPAddress agent_ip{192,168,9,100};

    #define RCCHECK(fn) { if(fn != RCL_RET_OK) { return false; }}

    bool createEntities();

    void destroyEntities();

};

#endif //BEERCRATEGRIPPER_WIFI_HPP