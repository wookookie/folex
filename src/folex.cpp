/******************************************************************************
* Copyright 2022 Hyunwook Choi
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

#include "folex.hpp"


FolexTask::FolexTask()
{}

FolexTask::~FolexTask()
{}

void FolexTask::threadCreate()
{
  pthread_create(&th_actuator_value_, NULL, &FolexTask::threadActuatorValue, this);
  pthread_create(&th_print_, NULL, &FolexTask::threadPrintValue, this);
  pthread_create(&th_trajectory_, NULL, &FolexTask::threadTrajectory, this);
}

void FolexTask::threadJoin()
{
  pthread_join(th_actuator_value_, NULL);
  pthread_join(th_print_, NULL);
  pthread_join(th_trajectory_, NULL);
}

void *FolexTask::threadActuatorValue(void *arg)
{
  ((FolexTask *)arg)->actuator();
  return NULL;
}

void *FolexTask::threadPrintValue(void *arg)
{
  ((FolexTask *)arg)->print();
  return NULL;
}

void *FolexTask::threadTrajectory(void *arg)
{
  ((FolexTask *)arg)->trajectory();
  return NULL;
}


Folex::Folex()
{}

Folex::~Folex()
{}

void Folex::actuator()
{
  Actuator *p_actuator = new Actuator();

  p_actuator->initActuator();
  p_actuator->enableActuator(Joint::JOINT_ALL);

  float _target_pos = 0;
  float _target_vel = 0;
  uint16_t kLimitVelocity = 250;
  uint16_t waypoint_index = 0;
  uint8_t kTestJointNum = Joint::JointNumber::JOINT_6;

  while (true)
  {
    if (Joint::trajectory_gen == true)
    {
      std::vector<Waypoint>::iterator it_joint_waypoint;
      for (it_joint_waypoint = Joint::joint_waypoint.begin(); it_joint_waypoint != Joint::joint_waypoint.end(); it_joint_waypoint++)
      {
        _target_pos = it_joint_waypoint->position;
        _target_vel = it_joint_waypoint->velocity;

        // Limit
        if (abs(_target_vel) > kLimitVelocity)
        {
          _target_vel = copysign(kLimitVelocity, _target_vel);
        }

        // Convert value to CW
        if (_target_vel < 0)
        {
          _target_vel = 1024 + abs(_target_vel);  // AX
        }

        p_actuator->readPresentAngle();
        p_actuator->readPresentVelocity();

        Joint::target_angle_value[kTestJointNum] = _target_pos;
        Joint::target_velocity_value[kTestJointNum] = _target_vel;

        p_actuator->writeTargetVelocity();
      }

      Joint::joint_waypoint.clear();
      Joint::trajectory_gen = false;
    }
    else
    {
      p_actuator->readPresentAngle();
      p_actuator->readPresentVelocity();
      p_actuator->writeTargetVelocity();
    }
  }
}

void Folex::print()
{
  uint32_t _count = 1;
  while (true)
  {
    // Count
    std::cout << "COUNT: " << _count << "\t";
    for (uint8_t i = Joint::JOINT_1; i < Joint::JOINT_ALL; i++)
    {
      std::cout << Joint::present_angle_value[i] << "\t";
    }

    std::cout << "\t";

    // Present velocity
    for (uint8_t i = Joint::JOINT_1; i < Joint::JOINT_ALL; i++)
    {
      if (Joint::present_velocity_value[i] > 1023)
      {
        int32_t pv = -1 * (Joint::present_velocity_value[i] - 1024);
        std::cout << pv << "\t";
      }
      else
      {
        std::cout << Joint::present_velocity_value[i] << "\t";
      }
    }

    std::cout << "\t";

    // Target velocity
    for (uint8_t i = Joint::JOINT_1; i < Joint::JOINT_ALL; i++)
    {
      if (Joint::target_velocity_value[i] > 1023)
      {
        int32_t tv = -1 * (Joint::target_velocity_value[i] - 1024);
        std::cout << tv << "\t";
      }
      else
      {
        std::cout << Joint::target_velocity_value[i] << "\t";
      }
    }

    // Target position
    // for (uint8_t i = Joint::JOINT_1; i < Joint::JOINT_ALL; i++)
    // {
    //   std::cout << Joint::target_angle_value[i] << "\t";
    // }

    std::cout << std::endl;

    _count++;
    nanosleep(&ts_msec_100, NULL);
  }
}

void Folex::trajectory()
{
  JointTrajectory *p_trajectory = new JointTrajectory();

  Waypoint start_point, end_point;
  start_point.velocity = 0;
  start_point.acceleration = 0;
  end_point.velocity = 0;
  end_point.acceleration = 0;

  bool reverse = true;
  while (true)
  {
    if (Joint::trajectory_gen == false)
    {
      if (reverse == true)
      {
        start_point.position = Joint::present_angle_value[Joint::JOINT_6];
        end_point.position = 512;
        reverse = false;
      }
      else
      {
        start_point.position = Joint::present_angle_value[Joint::JOINT_6];
        end_point.position = 700;
        reverse = true;
      }

      p_trajectory->generateTrajectory(start_point, end_point, 2.0);
      Joint::trajectory_gen = true;
    }

    nanosleep(&ts_msec_10, NULL);
  }
}
