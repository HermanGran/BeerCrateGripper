//
// Created by Herman Hårstad Gran on 23/04/2026.
//

#ifndef BEERCRATEGRIPPER_WIFI_HPP
#define BEERCRATEGRIPPER_WIFI_HPP

#include <vector>

#include <ROS/RosNode.hpp>


class MicroRosConnection {
public:

    std::vector<RosNode*> nodes;  // All registered ROS components

    void registerNode(RosNode * node);

    void initWiFi(const char* ssid, const char* password, const IPAddress& ros_agent);

    static void initOTA(const char* hostName, const char* password);

    void update();

    static void updateOTA();

private:
    const char *ssid_ = nullptr;
    const char *password_ = nullptr;
    const size_t domain_id_ = 7;
    const size_t agent_port_ = 8888;
    IPAddress agent_ip_ ;

    // Static network configurations
    const IPAddress localIP{192, 168, 0, 104};
    const IPAddress gateway{192, 168, 0, 1};
    const IPAddress subnet{255, 255, 255, 0};


    bool createEntities();

    void destroyEntities();

    bool errorLight = false;

    enum State { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };

    rcl_node_t      node;
    rcl_allocator_t allocator;
    rclc_support_t  support;
    rclc_executor_t executor;
    State           state = WAITING_AGENT;

};

#endif //BEERCRATEGRIPPER_WIFI_HPP