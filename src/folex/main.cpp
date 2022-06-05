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
#include "folex.hpp"

// Declare the static variables
float Joint::present_angle[Joint::JOINT_ALL];
float Joint::present_velocity[Joint::JOINT_ALL];
float Joint::target_angle[Joint::JOINT_ALL];
float Joint::target_velocity[Joint::JOINT_ALL];
uint32_t Joint::present_velocity_raw[Joint::JOINT_ALL];
uint32_t Joint::present_angle_value[Joint::JOINT_ALL];
int32_t Joint::present_velocity_value[Joint::JOINT_ALL];
uint32_t Joint::target_angle_value[Joint::JOINT_ALL];
uint32_t Joint::target_velocity_value[Joint::JOINT_ALL];
bool Joint::trajectory_gen = false;
std::vector<Waypoint> Joint::joint_waypoint;

// Class pointer
Folex *p_folex;


void signalHandler(int signal)
{
  std::cout << std::endl;

  p_folex->actuatorDisable();
  std::cout << "[SIGNAL] Actuator disabled" << std::endl;

  exit(signal);
}

int main(int argc, char **argv)
{
  p_folex = new Folex();
  p_folex->init();

  // Install a signal handler
  signal(SIGINT, signalHandler);

  p_folex->threadCreate();
  p_folex->threadJoin();

  return 0;
}
