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

#ifndef FOLEX_HPP
#define FOLEX_HPP

#include <csignal>
#include <iostream>
#include <time.h>

#include "actuator.hpp"
#include "essential.hpp"
#include "kinematics.hpp"
#include "quadruped_task.hpp"
#include "trajectory.hpp"


class Folex : public QuadrupedTask
{
private:
  bool actuator_disable = false;

  Actuator *p_actuator;
  JointTrajectory *p_trajectory;

public:
  Folex();
  ~Folex();

  void init();
  static void signalHandler(int signal);

  void actuatorDisable();
  void actuatorRxTx();
  void actuatorTargetCommand();
  void print();
  void trajectory();
};

#endif  // FOLEX_HPP
