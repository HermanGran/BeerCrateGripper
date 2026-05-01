//
// Created by Herman Hårstad Gran on 23/04/2026.
//

#include <ROS/GripperNode.hpp>
#include <Debug/Logger.hpp>
#include <Gripper/Gripper.hpp>

GripperNode* GripperNode::instance = nullptr;

GripperNode::GripperNode(Gripper &gripper) : gripper_(gripper) {}

int GripperNode::executor_handles() {
    return 1;
}

bool GripperNode::init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) {
    instance = this;

    if (rclc_service_init_default(
        &service, node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(workcell_interfaces, srv, GripperCommand),
        "/gripper_command") != RCL_RET_OK) {
        logger.logf("Failed to create gripper service!");
        return false;
    }

    if (rclc_executor_add_service(
        executor, &service,
        &request, &response,
        &service_callback) != RCL_RET_OK) {
        logger.logf("Failed to add service to executor!");
        return false;
    }

    return true;
}

void GripperNode::fini(rcl_node_t* node) {
    const rcl_ret_t rc = rcl_service_fini(&service, node);
    if (rc != RCL_RET_OK) {
        logger.logf("rcl_service_fini failed: %d", rc);
    }
}

void GripperNode::service_callback(const void* req, void* res) {
    const auto* request  = (workcell_interfaces__srv__GripperCommand_Request*)req;
    auto* response = (workcell_interfaces__srv__GripperCommand_Response*)res;

    auto cmd = static_cast<Gripper::GripperAction>(request->command);
    logger.logf("Service request: command=%d", cmd);

    bool success = false;

    switch (cmd) {
        case Gripper::GripperAction::HOMING: // 0
            logger.logf("Gripper homing...");
            instance->gripper_.homing();
            success = true;
            logger.logf("Done.");
            break;
        case Gripper::GripperAction::LATCH: // 1
            logger.logf("Gripper latching...");
            success = instance->gripper_.latch();
            logger.logf(success ? "Latched!" : "Latch failed!");
            break;
        case Gripper::GripperAction::HOME: // 2
            logger.logf("Gripper releasing...");
            instance->gripper_.home();
            success = true;
            logger.logf("Done.");
            break;
        case Gripper::GripperAction::IDLE: // 3
            logger.logf("Gripper moving to idle position...");
            instance->gripper_.idlePos();
            success = true;
            logger.logf("Done.");
            break;

        default:
            logger.logf("Unknown command: %d", request->command);
            break;
    }

    response->success = success;

    const char* msg = success ? "OK" : "Latch not detected";
    snprintf(instance->response_msg_buf, sizeof(instance->response_msg_buf), "%s", msg);

    response->message.data = instance->response_msg_buf;
    response->message.size = strlen(instance->response_msg_buf);
    response->message.capacity = sizeof(instance->response_msg_buf);

    logger.logf("Response: success=%d, msg=%s", success, instance->response_msg_buf);
}
