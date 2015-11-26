/*
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this file implements the library to control a brushless gimbal motor with the LXRobotics Brushless Motorshield
 * @file LXR_Brushless_Motorshield_Gimbal_Space_Vector.h
 * @license Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0) ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */ 

#ifndef LXR_BRUSHLESS_MOTORSHIELD_GIMBAL_SPACE_VECTOR_H_
#define LXR_BRUSHLESS_MOTORSHIELD_GIMBAL_SPACE_VECTOR_H_

/* INCLUDES */

#include "SpaceVectorPWMGenerator.h"

/* TYPEDEFS */

typedef enum {FORWARD=0, BACKWARD=1} E_DIRECTION;

/* PROTOTYPES */
	
class LXR_Brushless_Motorshield_Gimbal_Space_Vector 
{
public:
        /** 
	 * @brief Constructor
	 */
	LXR_Brushless_Motorshield_Gimbal_Space_Vector();

	/** 
	 * @brief initializes this library
	 */	
	void begin();
	
	/**
	 * @brief performs one step in the desired direction
	 */
	void step();
	
	/**
	 * @brief sets the direction for the brushless motor
	 */
	void set_direction(E_DIRECTION const direction);

  SpaceVectorPWMGenerator svpwm_gen_;
	
};

#endif /* LXR_BRUSHLESS_MOTORSHIELD_GIMBAL_H_ */
