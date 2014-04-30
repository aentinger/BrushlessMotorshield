/*
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this file implements the library to control a brushless gimbal motor with the LXRobotics Brushless Motorshield
 * @file LXR_Brushless_Motorshield_Gimbal.h
 * @license Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0) ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */ 

#ifndef LXR_BRUSHLESS_MOTORSHIELD_GIMBAL_H_
#define LXR_BRUSHLESS_MOTORSHIELD_GIMBAL_H_

typedef enum {FORWARD=0, BACKWARD=1} E_DIRECTION;
	
class LXR_Brushless_Motorshield_Gimbal {
public:
	/** 
	 * @brief initializes this library
	 */	
	static void begin();
	
	/**
	 * @brief performs one step in the desired direction
	 */
	static void step();
	
	/**
	 * @brief sets the direction for the brushless motor
	 */
	static void set_direction(E_DIRECTION const dir);
	
private:
	/** 
	 * @brief Constructor
	 */
	LXR_Brushless_Motorshield_Gimbal() { }
		
	/** 
	 * @brief Destructor
	 */
	~LXR_Brushless_Motorshield_Gimbal() { }
};

#endif /* LXR_BRUSHLESS_MOTORSHIELD_GIMBAL_H_ */
