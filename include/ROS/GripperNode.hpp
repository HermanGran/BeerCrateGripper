//
// Created by Herman Hårstad Gran on 23/04/2026.
//

#ifndef BEERCRATEGRIPPER_GRIPPERNODE_HPP
#define BEERCRATEGRIPPER_GRIPPERNODE_HPP

#include <ROS/RosNode.hpp>
#include <Gripper/Gripper.hpp>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>

class GripperNode : public RosNode {
public:
    explicit GripperNode(Gripper &gripper);

    int executor_handles() override;

    bool init(rcl_node_t *node, rclc_executor_t *executor) override;

private:
    Gripper &gripper_;
    rcl_publisher_t    current_pub;
    rcl_subscription_t command_sub;
    rcl_timer_t        timer;
    rcl_timer_t*       timer_ptr = &timer;
    rclc_support_t*    support_ptr = nullptr;

    std_msgs__msg__Float32 current_msg;
    std_msgs__msg__Bool    command_msg;


    static GripperNode* instance;

};

#endif //BEERCRATEGRIPPER_GRIPPERNODE_HPP