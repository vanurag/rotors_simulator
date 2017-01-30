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


#ifndef ROTORS_JOY_INTERFACE_JOY_POS_H_
#define ROTORS_JOY_INTERFACE_JOY_POS_H_

#include <thread>
#include <chrono>

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

struct Axes {
  int roll;
  int pitch;
};

struct Buttons {
  int takeoff;
  int land;
  int ctrl_enable;
  int ctrl_mode;
  int yaw_left;
  int yaw_right;
  int thrust_up;
  int thrust_down;
};

struct Max {
  double roll;
  double pitch;
  double yaw;
  double thrust;
};

class JoyPos {
  typedef sensor_msgs::Joy::_buttons_type ButtonType;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh;
  ros::Publisher ctrl_pub_;
  ros::Subscriber joy_sub_;

  std::string namespace_;

  Axes axes_;
  Buttons buttons_;

  Eigen::Vector3d desired_position_;
  double desired_yaw_;
  trajectory_msgs::MultiDOFJointTrajectory control_msg_;
  sensor_msgs::Joy current_joy_;

  Max max_;

  double current_yaw_vel_;
  double v_yaw_step_;

  void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
  void Publish();

 public:
  JoyPos();
};

#endif // ROTORS_JOY_INTERFACE_JOY_POS_H_
