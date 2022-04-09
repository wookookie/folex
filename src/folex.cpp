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


void *threadActuatorValue(void *arg)
{
  float _target_pos = 0;
  float _target_vel = 0;
  uint16_t kLimitVelocity = 250;
  uint16_t waypoint_index = 0;
  uint8_t kTestJointNum = Joint::JointNumber::JOINT_6;

  while (true)
  {
    if (trajectory_gen == true)
    {
      std::vector<Waypoint>::iterator it_joint_waypoint;
      for (it_joint_waypoint = trajectory.joint_waypoint.begin(); it_joint_waypoint != trajectory.joint_waypoint.end(); it_joint_waypoint++)
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

        actuator.readPresentAngle();
        actuator.readPresentVelocity();

        Joint::target_angle_value[kTestJointNum] = _target_pos;
        Joint::target_velocity_value[kTestJointNum] = _target_vel;

        actuator.writeTargetVelocity();
      }

      trajectory.joint_waypoint.clear();
      trajectory_gen = false;
    }
    else
    {
      actuator.readPresentAngle();
      actuator.readPresentVelocity();
      actuator.writeTargetVelocity();
    }
  }
}

void *threadPrintValue(void *arg)
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

void *threadTrajectory(void *arg)
{
  Waypoint start_point, end_point;
  start_point.velocity = 0;
  start_point.acceleration = 0;
  end_point.velocity = 0;
  end_point.acceleration = 0;

  bool reverse = true;
  while (true)
  {
    if (trajectory_gen == false)
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

      trajectory.generateTrajectory(start_point, end_point, 2.0);
      trajectory_gen = true;
    }

    nanosleep(&ts_msec_10, NULL);
  }
}


int main()
{
  actuator.initActuator();
  actuator.enableActuator(0);

  pthread_create(&th_actuator_value, NULL, threadActuatorValue, NULL);
  pthread_create(&th_trajectory, NULL, threadTrajectory, NULL);
  pthread_create(&th_print, NULL, threadPrintValue, NULL);

  pthread_join(th_actuator_value, NULL);
  pthread_join(th_trajectory, NULL);
  pthread_join(th_print, NULL);

  return 0;
}
