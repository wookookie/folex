//

#ifndef KINEMATICS_H
#define KINEMATICS_H

// For using abs()
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <math.h>


class Kinematics
{
public:
  void solveLegIK(float x, float y, float z);

  float upper_leg_joint;
  float lower_leg_joint;


private:
  float upper_leg_length = 67.5;          // 67.5mm
  float upper_leg_joint_offset = 22.5;    // 22.5mm
  float lower_leg_length = 82.35;         // 82.35mm


  float rad2deg = 180 / M_PI;







};

#endif // KINEMATICS_H
