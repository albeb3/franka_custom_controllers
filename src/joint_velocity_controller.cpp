// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_custom_controllers/joint_velocity_controller.h>

#include <cmath>
#include <memory>

#include "std_msgs/Float64MultiArray.h"

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "franka_custom_controllers/mission_manager.hpp"


namespace franka_custom_controllers {

bool JointVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
    task_priority_ = std::make_unique<task_priority>(node_handle);                                         
    return task_priority_->init(robot_hardware);
}

void JointVelocityController::starting(const ros::Time& /* time */) {
    task_priority_->starting();
}

void JointVelocityController::update(const ros::Time& /* time */,
                                            const ros::Duration& /* period */) {
    task_priority_->update();
   
  
}

void JointVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_custom_controllers

PLUGINLIB_EXPORT_CLASS(franka_custom_controllers::JointVelocityController,
                       controller_interface::ControllerBase)
