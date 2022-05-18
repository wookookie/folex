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

#include "trajectory.hpp"


JointTrajectory::JointTrajectory()
{}

JointTrajectory::~JointTrajectory()
{}

// Solve x vector
void JointTrajectory::calculateCoefficient(Waypoint start, Waypoint end, float time)
{
  Eigen::Matrix<float, 6, 6> A;
  A << 0, 0, 0, 0, 0, 1,
      pow(time, 5), pow(time, 4), pow(time, 3), pow(time, 2), time, 1,
      0, 0, 0, 0, 1, 0,
      5 * pow(time, 4), 4 * pow(time, 3), 3 * pow(time, 2), 2 * time, 1, 0,
      0, 0, 0, 2, 0, 0,
      20 * pow(time, 3), 12 * pow(time, 2), 6 * time, 2, 0, 0;

  Eigen::Matrix<float, 6, 1> b;
  b << start.position, end.position, start.velocity, end.velocity, start.acceleration, end.acceleration;

  // Decomposition
  Eigen::FullPivLU<Eigen::MatrixXf> dec(A);
  coefficient = dec.solve(b);
}

void JointTrajectory::calculateWaypoint(float tick)
{
  waypoint.position =
    coefficient(0) * pow(tick, 5) +
    coefficient(1) * pow(tick, 4) +
    coefficient(2) * pow(tick, 3) +
    coefficient(3) * pow(tick, 2) +
    coefficient(4) * pow(tick, 1) +
    coefficient(5);
  
  waypoint.velocity = 
    5 * coefficient(0) * pow(tick, 4) +
    4 * coefficient(1) * pow(tick, 3) +
    3 * coefficient(2) * pow(tick, 2) +
    2 * coefficient(3) * pow(tick, 1) +
    coefficient(4);

  waypoint.acceleration =
    20 * coefficient(0) * pow(tick, 3) +
    12 * coefficient(1) * pow(tick, 2) +
    6  * coefficient(2) * pow(tick, 1) +
    2  * coefficient(3);

  // Save waypoint
  Joint::joint_waypoint.push_back(waypoint);
}

void JointTrajectory::generateTrajectory(Waypoint start, Waypoint end, float time)
{
  // auto start_time = std::chrono::high_resolution_clock::now();

  calculateCoefficient(start, end, time);

  float tick = 0.0F;
  uint16_t tick_index = 0;
  tick_index = (time / control_period) + 1;

  if (fmod(time, control_period) > 0)
  {
    tick_index++;
  }

  for (uint16_t i = 0; i < tick_index; i++)
  {    
    calculateWaypoint(tick);
    tick += control_period;
  }

  // DEBUG
  // auto duration_time = std::chrono::high_resolution_clock::now() - start_time;
  // std::cout << "Generate time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(duration_time).count() * 1e-9 << std::endl;

  // uint16_t waypoint_index = 0;
  // std::vector<Waypoint>::iterator it_joint_waypoint;

  // for (it_joint_waypoint = joint_waypoint.begin(); it_joint_waypoint != joint_waypoint.end(); it_joint_waypoint++)
  // {
  //   std::cout << "WP: " << waypoint_index << "\t"
  //             << "POS: " << it_joint_waypoint->position << "\t"
  //             << "VEL: " << it_joint_waypoint->velocity << "\t"
  //             << "ACC: " << it_joint_waypoint->acceleration << "\t" << std::endl;

  //   waypoint_index++;
  // }

  // it_joint_waypoint = joint_waypoint.begin();
  // std::cout << "TRAJECTORY START >>\t"
  //           << "POS: " << it_joint_waypoint->position << "\t"
  //           << "VEL: " << it_joint_waypoint->velocity << "\t"
  //           << "ACC: " << it_joint_waypoint->acceleration << "\t" << std::endl;

  // it_joint_waypoint = joint_waypoint.end();
  // std::cout << "TRAJECTORY END >>\t"
  //           << "POS: " << it_joint_waypoint->position << "\t"
  //           << "VEL: " << it_joint_waypoint->velocity << "\t"
  //           << "ACC: " << it_joint_waypoint->acceleration << "\t" << std::endl;
}
