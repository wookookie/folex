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

#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <chrono>
#include <vector>

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif
#include <eigen3/Eigen/Eigen>

#include "essential.hpp"


class JointTrajectory
{
private:
  float control_period = 0.036F;
  Waypoint waypoint;
  Eigen::Matrix<float, 6, 1> coefficient;

  void calculateCoefficient(Waypoint start, Waypoint end, float time);
  void calculateWaypoint(float tick);

public:
  JointTrajectory();
  ~JointTrajectory();
  void generateTrajectory(Waypoint start, Waypoint end, float time);
};

#endif  // TRAJECTORY_HPP
