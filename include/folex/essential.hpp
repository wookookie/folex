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


class Joint
{
public:
  enum JointNumber
  {
    JOINT_ALL,
    JOINT_1, JOINT_2, JOINT_3,
    JOINT_4, JOINT_5, JOINT_6,
    JOINT_7, JOINT_8, JOINT_9,
    JOINT_10, JOINT_11, JOINT_12
  };

  /* Angle */
  // Radian
  static float present_angle[12];
  static float present_velocity[12];
  static float target_angle[12];
  static float target_velocity[12];
  // Dynamixel value
  static uint32_t present_angle_value[12];
  static uint32_t present_velocity_value[12];
  static uint32_t target_angle_value[12];
  static uint32_t target_velocity_value[12];
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
