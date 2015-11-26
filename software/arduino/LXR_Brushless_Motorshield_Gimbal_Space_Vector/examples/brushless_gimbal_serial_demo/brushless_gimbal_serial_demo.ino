/*
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief demonstration of control of a brushless gimbal motor with the LXRobotics brushless motorshield using only one timer and two interrupts
 * @file brushless_gimbal_serial_demo.ino
 * @license Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0) ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */ 

/* INCLUDES */

#include "LXR_Brushless_Motorshield_Gimbal_Space_Vector.h"

/* GLOBAL VARIABLES */

LXR_Brushless_Motorshield_Gimbal_Space_Vector gimbal;

/* CODE */

void setup() 
{
  gimbal.begin();
  
  gimbal.set_direction(FORWARD);

  Serial.begin(115200);
}

/*
void loop() 
{
  int delay_time_ms = 1;
  
  gimbal.step();
  
  delay(delay_time_ms);
}
*/

void loop() 
{
  while(Serial.available())
  {
    if(Serial.read() == 's')
    {
      gimbal.step();

      char buf[32];
      sprintf(buf, "T1 = %d, T2 = %d, V1 = %d, V2 = %d", gimbal.svpwm_gen_.get_t1(), gimbal.svpwm_gen_.get_t2(), gimbal.svpwm_gen_.get_t1_vect(), gimbal.svpwm_gen_.get_t2_vect());
      Serial.println(buf);
    }
  }
}
