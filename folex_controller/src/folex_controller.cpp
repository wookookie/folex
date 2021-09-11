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

#include "folex_controller.h"


FolexController::FolexController()
{
  present_joint_value_sub = nh.subscribe("opencr/joint_state", 10, &FolexController::callbackJointState, this);

  joint_states_pub = nh.advertise<sensor_msgs::JointState>("target_joint", 10); //?

  joint_names.push_back("joint1");
  joint_names.push_back("joint2");
  joint_names.push_back("joint3");
  joint_names.push_back("joint4");
  joint_names.push_back("joint5");
  joint_names.push_back("joint6");
  joint_names.push_back("joint7");
  joint_names.push_back("joint8");
  joint_names.push_back("joint9");
  joint_names.push_back("joint10");
  joint_names.push_back("joint11");
  joint_names.push_back("joint12");
}

FolexController::~FolexController()
{}

void FolexController::publishJointStates(float joint_value[], float joint_speed[])
{
  joint_state_msg.header.stamp = ros::Time::now();
  joint_state_msg.name.resize(joint_names.size());
  joint_state_msg.position.resize(joint_names.size());
  joint_state_msg.velocity.resize(joint_names.size());

  joint_state_msg.name = joint_names;

  // Save the target joint value to JointState message
  for (uint8_t i = 0; i < joint_names.size(); i++)
  {
    joint_state_msg.position[i] = joint_value[i];
    joint_state_msg.velocity[i] = joint_speed[i];
  }
  
  joint_states_pub.publish(joint_state_msg);
}

void FolexController::callbackJointState(const sensor_msgs::JointState::ConstPtr &msg)
{
  float pos[12];

  for (uint8_t i = 0; i < 12; i++)
  {
    pos[i] = msg->position[i];
    // ROS_INFO("Receive joint value : %f", pos[i]);
  }
}

void FolexController::kinematicsTest()
{
  float joint_angle[12];

  Eigen::Vector3f target_foot_position[4];

  // TEST POSITIONS
  target_foot_position[0](0, 0) = -8.7864;
  target_foot_position[0](1, 0) = 57.127;
  target_foot_position[0](2, 0) = -156.9551;

  target_foot_position[1](0, 0) = -8.7864;
  target_foot_position[1](1, 0) = -83.5141;
  target_foot_position[1](2, 0) = -144.6506;

  target_foot_position[2](0, 0) = 8.7864;
  target_foot_position[2](1, 0) = 57.127;
  target_foot_position[2](2, 0) = -156.9551;

  target_foot_position[3](0, 0) = 7.425;
  target_foot_position[3](1, 0) = 0;
  target_foot_position[3](2, 0) = -152.2739;

  std::cout << std::endl;
  std::cout << "Value XYZ [LF] : " << target_foot_position[0](0, 0) << "  " << target_foot_position[0](1, 0) << "  " << target_foot_position[0](2, 0) << std::endl;
  std::cout << "Value XYZ [RF] : " << target_foot_position[1](0, 0) << "  " << target_foot_position[1](1, 0) << "  " << target_foot_position[1](2, 0) << std::endl;
  std::cout << "Value XYZ [LH] : " << target_foot_position[2](0, 0) << "  " << target_foot_position[2](1, 0) << "  " << target_foot_position[2](2, 0) << std::endl;
  std::cout << "Value XYZ [RH] : " << target_foot_position[3](0, 0) << "  " << target_foot_position[3](1, 0) << "  " << target_foot_position[3](2, 0) << std::endl;

  kinematics.solveIK(joint_angle, target_foot_position);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "folex_controller");
  ROS_INFO("Folex Controller initialize complete.");

  FolexController folex_controller;
  float joint_value[12] = {2048, 512, 512, 2048, 512, 512, 2048, 512, 512, 2048, 512, 512};
  float joint_speed[12] = {20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20};

  ros::Rate loop_rate(1);

  // TEST
  folex_controller.kinematicsTest();

  while (ros::ok())
  {
    folex_controller.publishJointStates(joint_value, joint_speed);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
