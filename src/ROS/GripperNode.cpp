//
// Created by Herman Hårstad Gran on 23/04/2026.
//

#include <ROS/GripperNode.hpp>
#include <Debug/Logger.hpp>



GripperNode::GripperNode(Gripper &gripper) : gripper_(gripper) {}

int GripperNode::executor_handles() {
    return 2;
}

bool GripperNode::init(rcl_node_t *node, rclc_executor_t *executor) {
    instance = this;

    if (rclc_publisher_init_default) {
        &current_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "gripper/current") != RCL_RET_OK) return false;
    }
}

