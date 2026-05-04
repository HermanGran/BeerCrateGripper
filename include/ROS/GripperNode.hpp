//
// Created by Herman Hårstad Gran on 23/04/2026.
//

#ifndef BEERCRATEGRIPPER_GRIPPERNODE_HPP
#define BEERCRATEGRIPPER_GRIPPERNODE_HPP

#include <ROS/RosNode.hpp>
#include <Gripper/Gripper.hpp>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>

#include <rclc/service.h>
#include <workcell_interfaces/srv/gripper_command.h>

class GripperNode final : public RosNode {
public:
    explicit GripperNode(Gripper &gripper);

    int executor_handles() override;

    bool init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) override;
    void fini(rcl_node_t* node) override;

private:
    Gripper &gripper_;

    rcl_service_t service;
    workcell_interfaces__srv__GripperCommand_Request  request;
    workcell_interfaces__srv__GripperCommand_Response response;

    std_msgs__msg__Float32 current_msg;
    std_msgs__msg__Bool    command_msg;

    static GripperNode* instance;

    bool serviceInited_ = false;
    char response_msg_buf[64];

    static void service_callback(const void* req, void* res);

};



#endif //BEERCRATEGRIPPER_GRIPPERNODE_HPP