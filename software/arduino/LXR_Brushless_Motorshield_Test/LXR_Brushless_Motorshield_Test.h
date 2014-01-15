/**
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this module implements a test library for the brushless motor shield
 * @file LXR_Brushless_Motorshield_Test.h
 * @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */
 
#ifndef LXR_BRUSHLESS_MOTORSHIELD_TEST_H_
#define LXR_BRUSHLESS_MOTORSHIELD_TEST_H_
 
#include <Arduino.h>
 
typedef enum {O_LOW = 0, O_HIGH = 1, O_HIGH_IMPEDANCE = 2} E_OUTPUT_STATE;
 
class LXR_Brushless_Motorshield_Test {
 public:
	/**
	 * @brief initializes the lxr brushless motorshield test library
	 */ 
	static void begin();
	
	/**
	 * @brief set the output state for U
	 * @param state output state to set
	 */
	static void set_state_U(E_OUTPUT_STATE const state);
	
	/**
	 * @brief set the output state for V
	 * @param state output state to set
	 */
	static void set_state_V(E_OUTPUT_STATE const state);
	
	/**
	 * @brief set the output state for W
	 * @param state output state to set
	 */
	static void set_state_W(E_OUTPUT_STATE const state); 
 };
 
#endif