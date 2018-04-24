/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "rotors_joy_interface/joy_pos.h"

#include <mav_msgs/default_topics.h>

JoyPos::JoyPos() : pnh("~") {
  ctrl_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory> (
    mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  control_msg_.header.stamp = ros::Time::now();
  desired_position_.x() = 0.0;
  desired_position_.y() = 0.0;
  desired_position_.z() = 1.0;
  desired_yaw_ = 0.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position_,
      desired_yaw_, &control_msg_);

  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh_.getNamespace().c_str(),
           desired_position_.x(),
           desired_position_.y(),
           desired_position_.z());
  Publish();

  pnh.param("axis_roll_", axes_.roll, 0);
  pnh.param("axis_pitch_", axes_.pitch, 1);

  pnh.param("max_roll", max_.roll, 0.05);  // [m]
  pnh.param("max_pitch", max_.pitch, 0.05);  // [m]
  pnh.param("max_yaw", max_.yaw, 0.1);  // [rad]
  pnh.param("max_thrust", max_.thrust, 0.2);  // [m]

  pnh.param("button_thrust_down_", buttons_.thrust_down, 1);
  pnh.param("button_thrust_up_", buttons_.thrust_up, 2);
  pnh.param("button_yaw_left_", buttons_.yaw_left, 3);
  pnh.param("button_yaw_right_", buttons_.yaw_right, 4);
  pnh.param("button_ctrl_enable_", buttons_.ctrl_enable, 5);
  pnh.param("button_ctrl_mode_", buttons_.ctrl_mode, 10);
  pnh.param("button_takeoff_", buttons_.takeoff, 7);
  pnh.param("button_land_", buttons_.land, 8);

  namespace_ = nh_.getNamespace();
  joy_sub_ = nh_.subscribe("joy", 10, &JoyPos::JoyCallback, this);
}

void JoyPos::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  current_joy_ = *msg;

  Eigen::Vector3d forward_direction(cos(desired_yaw_), sin(desired_yaw_), 0);
  Eigen::Vector3d sideward_direction(-sin(desired_yaw_), cos(desired_yaw_), 0);
  Eigen::Vector3d upward_direction(0.0, 0.0, 1.0);
  desired_position_ += msg->axes[axes_.pitch] * max_.pitch * forward_direction;
  desired_position_ += msg->axes[axes_.roll] * max_.roll * sideward_direction;

  if (msg->buttons[buttons_.yaw_left]) {
    desired_yaw_ += max_.yaw;
  }
  else if (msg->buttons[buttons_.yaw_right]) {
    desired_yaw_ -= max_.yaw;
  }

  if (msg->buttons[buttons_.thrust_up]) {
    desired_position_ += max_.thrust * upward_direction;
  }
  else if (msg->buttons[buttons_.thrust_down]) {
    desired_position_ -= max_.thrust * upward_direction;
  }

  ros::Time update_time = ros::Time::now();
  control_msg_.header.stamp = update_time;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position_,
      desired_yaw_, &control_msg_);

  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh_.getNamespace().c_str(),
           desired_position_.x(),
           desired_position_.y(),
           desired_position_.z());
  Publish();
}

void JoyPos::Publish() {
  ctrl_pub_.publish(control_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_joy_interface");
  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  }
  else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  JoyPos joy;
  ros::spin();

  return 0;
}
