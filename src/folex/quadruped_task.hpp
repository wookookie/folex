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

#ifndef QUADRUPED_TASK_HPP
#define QUADRUPED_TASK_HPP

#include <pthread.h>


class QuadrupedTask
{
protected:
  pthread_t th_actuator_rxtx_;
  pthread_t th_actuator_target_;
  pthread_t th_trajectory_;
  pthread_t th_socket_recv_;
  pthread_t th_socket_send_;

public:
  QuadrupedTask();
  virtual ~QuadrupedTask();

  virtual void actuatorRxTx() = 0;
  virtual void actuatorTargetCommand() = 0;
  virtual void socketReceive() = 0;
  virtual void socketSend() = 0;
  virtual void trajectory() = 0;

  void threadCreate();
  void threadJoin();
  static void *threadActuatorRxTx(void *arg);
  static void *threadActuatorTargetCommand(void *arg);
  static void *threadSocketReceive(void *arg);
  static void *threadSocketSend(void *arg);
  static void *threadTrajectory(void *arg);
};

#endif  // QUADRUPED_TASK_HPP
