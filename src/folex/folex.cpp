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
#include "essential.hpp"
#include "folex.hpp"


Folex::Folex()
{}

Folex::~Folex()
{}

void Folex::init()
{
  p_actuator = new Actuator();
  p_socket_ = new SocketClient();
  p_trajectory = new JointTrajectory();

  p_actuator->initActuator();
  p_actuator->enableActuator(Joint::JOINT_ALL);
  p_socket_->connectServer();
}

void Folex::actuatorDisable()
{
  actuator_disable = true;
  nanosleep(&ts_msec_100, NULL);
}

void Folex::actuatorRxTx()
{
  while (true)
  {
    p_actuator->readPresentAngle();
    p_actuator->readPresentVelocity();

    if (actuator_disable == true)
    {
      p_actuator->writeTargetVelocityZero();
    }
    else
    {
      p_actuator->writeTargetVelocity();
    }
  }
}

void Folex::actuatorTargetCommand()
{
  while (true)
  {
    // TO-DO
    nanosleep(&ts_msec_1, NULL);
  }
}

void Folex::socketReceive()
{
  while (true)
  {
    p_socket_->receiveData();
  }
}

void Folex::socketSend()
{
  while (true)
  {
    p_socket_->sendData();
  }
}

void Folex::trajectory()
{
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
