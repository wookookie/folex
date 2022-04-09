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

#ifndef ESSENTIAL_HPP
#define ESSENTIAL_HPP

#include <iostream>


// Sleep time
static struct timespec ts_msec_1 = {.tv_sec = 0, .tv_nsec = 1000000};
static struct timespec ts_msec_10 = {.tv_sec = 0, .tv_nsec = 10000000};
static struct timespec ts_msec_100 = {.tv_sec = 0, .tv_nsec = 100000000};
static struct timespec ts_sec_1 = {.tv_sec = 1, .tv_nsec = 0};

class Joint
{
public:
  enum JointNumber
  {
    JOINT_1, JOINT_2, JOINT_3,
    JOINT_4, JOINT_5, JOINT_6,
    JOINT_7, JOINT_8, JOINT_9,
    JOINT_10, JOINT_11, JOINT_12,
    JOINT_ALL,
  };

  /* Angle */
  // Radian
  static float present_angle[JOINT_ALL];
  static float present_velocity[JOINT_ALL];
  static float target_angle[JOINT_ALL];
  static float target_velocity[JOINT_ALL];
  // Dynamixel value
  static uint32_t present_angle_value[JOINT_ALL];
  static uint32_t present_velocity_value[JOINT_ALL];
  static uint32_t target_angle_value[JOINT_ALL];
  static uint32_t target_velocity_value[JOINT_ALL];
};

class Leg
{
public:
  // Dimension
  static constexpr float upper_leg_offset = -22.5F;    // 22.50 mm
  static constexpr float upper_leg_length = -67.5F;    // 67.50 mm
  static constexpr float lower_leg_length = -82.35F;   // 82.35 mm
};

#endif // ESSENTIAL_HPP
