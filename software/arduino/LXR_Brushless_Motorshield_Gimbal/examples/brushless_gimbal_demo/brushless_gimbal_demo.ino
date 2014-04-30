/*
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief demonstration of control of a brushless gimbal motor with the LXRobotics brushless motorshield using only one timer and two interrupts
 * @file brushless_gimbal_demo.ino
 * @license Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0) ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */ 
 
#include "LXR_Brushless_Motorshield_Gimbal.h"

void setup() {
  LXR_Brushless_Motorshield_Gimbal::begin();
}

void loop() {
  {
    int const steps = random(250,1000);
    delay(100);
    LXR_Brushless_Motorshield_Gimbal::set_direction(FORWARD);
    for(int i=0; i<steps; i++) {
      LXR_Brushless_Motorshield_Gimbal::step();
      delay(1);
    }
  }
  {
    int const steps = random(250,1000);
    delay(100);
    LXR_Brushless_Motorshield_Gimbal::set_direction(BACKWARD);
    for(int i=0; i<steps; i++) {
      LXR_Brushless_Motorshield_Gimbal::step();
      delay(1);
    }
  } 
}
