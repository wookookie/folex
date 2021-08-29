//

#include "folex_controller.h"


FolexController::FolexController()
{
  present_joint_value_sub = nh.subscribe("opencr/joint_state", 10, &FolexController::callbackJointState, this);

  joint_states_pub = nh.advertise<sensor_msgs::JointState>("target_joint", 10); //?
  
  // Test joints
  joint_names.push_back("joint1");
  joint_names.push_back("joint2");
  joint_names.push_back("joint3");
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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "folex_controller");
  ROS_INFO("Folex Controller initialize complete.");

  FolexController folex_controller;
  float joint_value[12] = {2048, 512, 512, 2048, 512, 512, 2048, 512, 512, 2048, 512, 512};
  float joint_speed[12] = {20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20};

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    folex_controller.publishJointStates(joint_value, joint_speed);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;  
}
