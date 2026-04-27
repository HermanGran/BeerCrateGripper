//
// Created by Herman Hårstad Gran on 23/04/2026.
//

#ifndef BEERCRATEGRIPPER_ROSNODE_HPP
#define BEERCRATEGRIPPER_ROSNODE_HPP

#include <rclc/executor.h>

class RosNode {
public:
    virtual ~RosNode() = default;

    virtual bool init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) = 0;
    virtual void fini(rcl_node_t* node) = 0;
    virtual int executor_handles() { return 0; } // How many subscribers/services/timers
};

#endif //BEERCRATEGRIPPER_ROSNODE_HPP