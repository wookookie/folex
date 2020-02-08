//

#include "folex_diagnosis.h"


FolexDiagnosis::FolexDiagnosis()
{
  pinMode(BDPIN_LED_USER_1, OUTPUT);

}

FolexDiagnosis::~FolexDiagnosis()
{

}

void FolexDiagnosis::init()
{
  
}

void FolexDiagnosis::showLedStatus()
{
  digitalWrite(BDPIN_LED_USER_1, LOW);
  delay(500);
  digitalWrite(BDPIN_LED_USER_1, HIGH);
}
