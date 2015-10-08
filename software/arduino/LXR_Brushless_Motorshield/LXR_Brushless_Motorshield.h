/**
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this file implements an arduino library to interface the LXRobotics Brushless Motorshield
 * @file LXR_Brushless_Motorshield.h
 * @license Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0) ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */

#ifndef LXR_BRUSHLESS_MOTORSHIELD_H_
#define LXR_BRUSHLESS_MOTORSHIELD_H_

#include <stdint.h>

typedef enum {FORWARD = 0, BACKWARD = 1} E_DIRECTION;

class LXR_Brushless_Motorshield {
public:
  /**
   * @brief initialize the brushless shield
   */
  static void begin();
  
  /**
   * @brief sets the speed of the brushless motor
   * @param speed value between 0 and 255, where 0 is stop and 255 is full speed
   */
  static void set_speed(uint8_t const speed);
  
  /**
   * @brief returns the current speed
   */
  static uint8_t get_speed();
  
  /**
   * @brief sets the direction of the brushless motor
   */
  static void set_direction(E_DIRECTION const dir);
  
  /**
   * @brief sets the direction of the brushless motor
   */
  static E_DIRECTION get_direction();

 /**
 * @brief returns the current state of the software (sartup mode, controlled mode)
 */

  static bool is_startup_complete(); 
};

#endif


