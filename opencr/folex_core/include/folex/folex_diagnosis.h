//

#ifndef FOLEX_DIAGNOSIS_H
#define FOLEX_DIAGNOSIS_H

#include <Arduino.h>

// OpenCR LED Number
#define LED_ROS                     23
#define LED_WORKING



class FolexDiagnosis
{
public:
  FolexDiagnosis();
  ~FolexDiagnosis();

  void init();
  void showLedStatus();


private:

};

#endif // FOLEX_DIAGNOSIS_H
