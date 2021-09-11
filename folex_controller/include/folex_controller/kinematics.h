/******************************************************************************
* Copyright 2021 Hyunwook Choi (Daniel Choi)
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

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <eigen3/Eigen/Eigen>

// DEBUG
#include <iostream>


namespace Folex
{
  class Kinematics
  {
  private:
    // Leg dimensions
    static constexpr float upper_leg_offset = -22.5f;    // 22.50 mm
    static constexpr float upper_leg_length = -67.5f;    // 67.50 mm
    static constexpr float lower_leg_length = -82.35f;   // 82.35 mm

  public:
    Kinematics() {}
    ~Kinematics() {}

    void solveIK(float (&joint_angle)[12], Eigen::Vector3f (&foot_position)[4])
    {
      float calculated_angle[12];

      for (uint8_t i = 0; i < 4; i++)
      {
        solveIK(calculated_angle[(i * 3)], calculated_angle[(i * 3) + 1], calculated_angle[(i * 3) + 2], foot_position[i]);
      }

      for (uint8_t i = 0; i < 12; i++)
      {
        joint_angle[i] = calculated_angle[i];
      }
    }

    static void solveIK(float &hip_angle, float &upper_leg_angle, float &lower_leg_angle, Eigen::Vector3f &foot_position)
    {
      hip_angle = (M_PI / 2) + atan2(foot_position(2, 0), foot_position(1, 0));

      // Rotation matrix
      Eigen::Matrix3f rotate_x;
      rotate_x <<
        1, 0, 0,
        0, cosf(-hip_angle), -sinf(-hip_angle),
        0, sinf(-hip_angle), cosf(-hip_angle);

      // Matrix multiplication
      Eigen::Vector3f rotated_position;
      rotated_position = rotate_x * foot_position;

      // DEBUG
      std::cout << std::endl;
      std::cout << rotated_position << std::endl;

      // Calculate the lower leg angle
      lower_leg_angle = -1 * acosf((pow(rotated_position(0, 0), 2) + pow(rotated_position(2, 0) - upper_leg_offset, 2) - pow(upper_leg_length, 2) - pow(lower_leg_length, 2)) / (2 * upper_leg_length * lower_leg_length));

      // Calculate the upper leg angle
      float alpha_angle = atan2(rotated_position(2, 0) - upper_leg_offset, rotated_position(0, 0));
      float beta_angle = atan2(lower_leg_length * sinf(-lower_leg_angle), upper_leg_length + (lower_leg_length * cosf(-lower_leg_angle)));
      upper_leg_angle = (M_PI / 2) - (alpha_angle - beta_angle);

      // DEBUG
      std::cout << std::endl;
      std::cout << "Hip angle : \t  " << hip_angle << "  " << hip_angle * (180 / M_PI) << std::endl;
      std::cout << "Upper leg angle : " << upper_leg_angle << "  " << upper_leg_angle * (180 / M_PI) << std::endl;
      std::cout << "Lower leg angle : " << lower_leg_angle << "  " << lower_leg_angle * (180 / M_PI) << std::endl;
    }
  };
}

#endif  // KINEMATICS_H
