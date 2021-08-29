/******************************************************************************
* Copyright 2021 Hyunwook Choi (Daniel Choi)
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
******************************************************************************/

#ifndef FOLEX_CONTROLLER_H
#define FOLEX_CONTROLLER_H

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>



class FolexController
{
private:
  ros::NodeHandle nh;

  ros::Subscriber present_joint_value_sub;

  sensor_msgs::JointState joint_state_msg;
  ros::Publisher joint_states_pub;

  std::vector<std::string> joint_names;


public:
  FolexController();
  ~FolexController();

  void publishJointStates(float joint_value[], float joint_speed[]);

  void callbackJointState(const sensor_msgs::JointState::ConstPtr &msg);
};

# endif  // FOLEX_CONTROLLER_H
