/**
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this file demonstrates how to use the brushless motor
 * @file brushless_demo.ino
 */

#include "LXR_Brushless_Motorshield.h"

void setup() {
  LXR_Brushless_Motorshield::begin();
  Serial.begin(115200);
}

void loop() {
  ramp_up();
  ramp_down();
}

/**
 * @brief increases the speed from 0 to 255
 */
void ramp_up() {
  for(int s = 20; s < 255; s+=5) {
    LXR_Brushless_Motorshield::set_speed(s);
    Serial.print("Speed = ");
    Serial.println(s);
    delay(100);
  }
}

/**
 * @brief reduces the speed from 255 to 0
 */
void ramp_down() {
  for(int s = 255; s > 20; s-=5) {
    LXR_Brushless_Motorshield::set_speed(s);
    Serial.print("Speed = ");
    Serial.println(s);
    delay(100);
  }
}


