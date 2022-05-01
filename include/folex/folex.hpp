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

#include <iostream>
#include <pthread.h>
#include <time.h>

#include "actuator.hpp"
#include "essential.hpp"
#include "kinematics.hpp"
#include "trajectory.hpp"


class FolexTask
{
protected:
  pthread_t th_actuator_value_;
  pthread_t th_trajectory_;
  pthread_t th_print_;

public:
  FolexTask();
  virtual ~FolexTask();

  virtual void actuator() = 0;
  virtual void print() = 0;
  virtual void trajectory() = 0;

  void threadCreate();
  void threadJoin();
  static void *threadActuatorValue(void *arg);
  static void *threadPrintValue(void *arg);
  static void *threadTrajectory(void *arg);
};

class Folex : public FolexTask
{
private:
  Actuator *p_actuator;
  JointTrajectory *p_trajectory;

public:
  Folex();
  ~Folex();

  void init();

  void actuator();
  void print();
  void trajectory();
};

#endif  // FOLEX_HPP
