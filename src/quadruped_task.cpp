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

#include "quadruped_task.hpp"


QuadrupedTask::QuadrupedTask()
{}

QuadrupedTask::~QuadrupedTask()
{}

void QuadrupedTask::threadCreate()
{
  pthread_create(&th_actuator_rxtx_, NULL, &QuadrupedTask::threadActuatorRxTx, this);
  pthread_create(&th_actuator_target_, NULL, &QuadrupedTask::threadActuatorTargetCommand, this);
  pthread_create(&th_print_, NULL, &QuadrupedTask::threadPrintValue, this);
  pthread_create(&th_trajectory_, NULL, &QuadrupedTask::threadTrajectory, this);
}

void QuadrupedTask::threadJoin()
{
  pthread_join(th_actuator_rxtx_, NULL);
  pthread_join(th_actuator_target_, NULL);
  pthread_join(th_print_, NULL);
  pthread_join(th_trajectory_, NULL);
}

void *QuadrupedTask::threadActuatorRxTx(void *arg)
{
  ((QuadrupedTask *)arg)->actuatorRxTx();
  return NULL;
}

void *QuadrupedTask::threadActuatorTargetCommand(void *arg)
{
  ((QuadrupedTask *)arg)->actuatorTargetCommand();
  return NULL;
}

void *QuadrupedTask::threadPrintValue(void *arg)
{
  ((QuadrupedTask *)arg)->print();
  return NULL;
}

void *QuadrupedTask::threadTrajectory(void *arg)
{
  ((QuadrupedTask *)arg)->trajectory();
  return NULL;
}
