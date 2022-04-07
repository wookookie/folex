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

#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <cmath>
#include <iostream>

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif
#include <eigen3/Eigen/Eigen>

#include "essential.hpp"


class Kinematics
{
private:
  // Rotation matrix
  Eigen::Matrix3f rotate_x_;
  Eigen::Vector3f rotated_position_;
  // Temperature angle
  float alpha_angle;
  float beta_angle;
  // Result angle
  float calculated_angle_[12];

public:
  Kinematics() {}
  ~Kinematics() {}

  void solveIK(float (&joint_angle)[12], Eigen::Vector3f (&foot_position)[4])
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      solveLegIK(calculated_angle_[(i * 3)], calculated_angle_[(i * 3) + 1], calculated_angle_[(i * 3) + 2], foot_position[i]);

      joint_angle[(i * 3)] = calculated_angle_[(i * 3)];
      joint_angle[(i * 3) + 1] = calculated_angle_[(i * 3) + 1];
      joint_angle[(i * 3) + 2] = calculated_angle_[(i * 3) + 2];
    }
  }

  void solveLegIK(float &hip_angle, float &upper_leg_angle, float &lower_leg_angle, Eigen::Vector3f &foot_position)
  {
    hip_angle = (M_PI / 2) + atan2(foot_position(2, 0), foot_position(1, 0));

    if (hip_angle > 0)
    {
      rotate_x_ <<
        1, 0, 0,
        0, cosf(-hip_angle), -sinf(-hip_angle),
        0, sinf(-hip_angle), cosf(-hip_angle);

      rotated_position_ = rotate_x_ * foot_position;
    }
    else
    {
      rotated_position_ = foot_position;
    }

    // Calculate the lower leg angle
    lower_leg_angle = -1 * acosf((pow(rotated_position_(0, 0), 2) + pow(rotated_position_(2, 0) - Leg::upper_leg_offset, 2) - pow(Leg::upper_leg_length, 2) - pow(Leg::lower_leg_length, 2))
                                / (2 * Leg::upper_leg_length * Leg::lower_leg_length));
    if (std::isnan(lower_leg_angle))
    {
      // No more proceed calculation
      return;
    }

    // Calculate the upper leg angle
    alpha_angle = atan2(rotated_position_(2, 0) - Leg::upper_leg_offset, rotated_position_(0, 0));
    beta_angle = atan2(Leg::lower_leg_length * sinf(-lower_leg_angle), Leg::upper_leg_length + (Leg::lower_leg_length * cosf(-lower_leg_angle)));
    upper_leg_angle = (M_PI / 2) - (alpha_angle - beta_angle);
  }
};

#endif // KINEMATICS_HPP
