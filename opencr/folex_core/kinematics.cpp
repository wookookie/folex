//

#include "kinematics.h"

void Kinematics::solveLegIK(float value_x, float value_y, float value_z)
{

  float cosineTheta2 = (pow(value_x, 2) + pow(value_z, 2) - pow(upper_leg_length, 2) - pow(lower_leg_length, 2)) / (2 * upper_leg_length * lower_leg_length);
  float theta2 = acos(cosineTheta2);

  float sineTheta2 = sqrt(1 - pow(cosineTheta2, 2));
  float angle_beta = atan2((lower_leg_length * sineTheta2), (upper_leg_length + lower_leg_length * cosineTheta2));

  float angle_gamma = 0;
  if (value_x >= 0)
  {
    angle_gamma = atanf(abs(value_x) / abs(value_z));
  }
  else if (value_x < 0)
  {
    angle_gamma = -1 * (atanf(abs(value_x) / abs(value_z)));
  }
  else
  {
    // TODO : Error log
    return;
  }

  float theta1 = 0;
  if (value_z >= 0)
  {
    theta1 = (M_PI / 2) - (angle_beta + angle_gamma);
  }
  else if (value_z < 0)
  {
    theta1 = angle_beta - angle_gamma;
  }
  else
  {
    // TODO : Error log
    return;
  }
  
  upper_leg_joint = -1 * theta1 * rad2deg;  // negative value (임시)
  lower_leg_joint = theta2 * rad2deg;
}