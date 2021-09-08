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
    float upper_leg_offset = -22.5;    // 22.50 mm
    float upper_leg_length = -67.5;    // 67.50 mm
    float lower_leg_length = -82.35;   // 82.35 mm

  public:
    Kinematics() {}
    ~Kinematics() {}

    void solveIK(float point_x, float point_y, float point_z)
    {
      float hip_angle = (M_PI / 2) + atan2(point_z, point_y);

      // Rotation matrix
      Eigen::Matrix3f rotate_x;
      rotate_x <<
        1, 0, 0,
        0, cosf(-hip_angle), -sinf(-hip_angle),
        0, sinf(-hip_angle), cosf(-hip_angle);

      // XYZ point vector
      Eigen::Vector3f point_vector;
      point_vector <<
        point_x,
        point_y,
        point_z;

      // Matrix multiplication
      Eigen::Vector3f result_vector;
      result_vector = rotate_x * point_vector;

      // Save the calculated vector value to the point variable
      point_x = result_vector(0);
      point_y = result_vector(1);
      point_z = result_vector(2);

      // DEBUG
      std::cout << std::endl;
      std::cout << result_vector << std::endl;

      // Calculate the lower leg angle
      float lower_leg_angle = -1 * acosf((pow(point_x, 2) + pow(point_z - upper_leg_offset, 2) - pow(upper_leg_length, 2) - pow(lower_leg_length, 2)) / (2 * upper_leg_length * lower_leg_length));

      // Calculate the upper leg angle
      float alpha_angle = atan2(point_z - upper_leg_offset, point_x);
      float beta_angle = atan2(lower_leg_length * sinf(-lower_leg_angle), upper_leg_length + (lower_leg_length * cosf(-lower_leg_angle)));
      float upper_leg_angle = (M_PI / 2) - (alpha_angle - beta_angle);

      // DEBUG
      std::cout << std::endl;
      std::cout << "Hip angle : \t  " << hip_angle << "  " << hip_angle * (180 / M_PI) << std::endl;
      std::cout << "Upper leg angle : " << upper_leg_angle << "  " << upper_leg_angle * (180 / M_PI) << std::endl;
      std::cout << "Lower leg angle : " << lower_leg_angle << "  " << lower_leg_angle * (180 / M_PI) << std::endl;
    }
  };
}

#endif  // KINEMATICS_H
