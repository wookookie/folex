//

#include "folex_core.h"

void setup()
{
  // Initialize ROS node handle
  nh.initNode();
  nh.getHardware()->setBaud(1000000);

  int led_pin_1 = BDPIN_LED_USER_1;
  pinMode(led_pin_1, OUTPUT);
  digitalWrite(led_pin_1, LOW);

  if (dynamixel_driver.init() == false)
  {
    int led_pin_2 = BDPIN_LED_USER_2;
    pinMode(led_pin_2, OUTPUT);
    digitalWrite(led_pin_2, LOW);
  }
  
  if (dynamixel_driver.writePosition() == false)
  {
    int led_pin_3 = BDPIN_LED_USER_3;
    pinMode(led_pin_3, OUTPUT);
    digitalWrite(led_pin_3, LOW);
  }
}

void loop()
{
  int led_pin_4 = BDPIN_LED_USER_4;
  pinMode(led_pin_4, OUTPUT);
  digitalWrite(led_pin_4, LOW);

  startLogMsg();
  delay(500);

  digitalWrite(led_pin_4, HIGH);

  sendLogMsg();
  delay(500);

  nh.spinOnce();
}


// Functions
void sendLogMsg()
{
  char msg[100];

  if (nh.connected())
  {
    sprintf(msg, "Connected to OpenCR Board!");
    nh.loginfo(msg);
  }
}

void startLogMsg()
{
  char msg[100];

  if (nh.connected())
  {
    sprintf(msg, "--- LOOP Start ---");
    nh.loginfo(msg);
  }
}
