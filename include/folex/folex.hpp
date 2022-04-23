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

#include <iostream>
#include <pthread.h>
#include <time.h>

#include "actuator.hpp"
#include "essential.hpp"
#include "kinematics.hpp"
#include "trajectory.hpp"

// Declare the static variables
float Joint::present_angle[Joint::JOINT_ALL];
float Joint::present_velocity[Joint::JOINT_ALL];
float Joint::target_angle[Joint::JOINT_ALL];
float Joint::target_velocity[Joint::JOINT_ALL];
uint32_t Joint::present_angle_value[Joint::JOINT_ALL];
uint32_t Joint::present_velocity_value[Joint::JOINT_ALL];
uint32_t Joint::target_angle_value[Joint::JOINT_ALL];
uint32_t Joint::target_velocity_value[Joint::JOINT_ALL];
bool Joint::trajectory_gen = false;
std::vector<Waypoint> Joint::joint_waypoint;


class Folex
{
private:
  pthread_t th_actuator_value_;
  pthread_t th_trajectory_;
  pthread_t th_print_;

public:
  Folex();
  ~Folex();
  void threadCreate();
  void threadJoin();
  static void *threadActuatorValue(void *arg);
  static void *threadPrintValue(void *arg);
  static void *threadTrajectory(void *arg);
};
