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
#include <pthread.h>
#include <time.h>

#include "actuator.hpp"
#include "essential.hpp"
#include "kinematics.hpp"
#include "trajectory.hpp"

// Declare the static variables
float Joint::present_angle[12];
float Joint::present_velocity[12];
float Joint::target_angle[12];
float Joint::target_velocity[12];
uint32_t Joint::present_angle_value[12];
uint32_t Joint::present_velocity_value[12];
uint32_t Joint::target_angle_value[12];
uint32_t Joint::target_velocity_value[12];

/* Actuator */
Actuator actuator;
// Thread
pthread_t th_actuator_value;

// Kinematics
Kinematics kinematics;
// Target value
float joint_angle[12];
Eigen::Vector3f target_foot_position[4];

/* Trajectory */
JointTrajectory trajectory;
// Thread
pthread_t th_trajectory;
// Trajectory generation indicator
bool trajectory_gen;

/* Print values */
pthread_t th_print;


void *threadActuatorValue(void *arg);
void *threadPrintValue(void *arg);
void *threadTrajectory(void *arg);
