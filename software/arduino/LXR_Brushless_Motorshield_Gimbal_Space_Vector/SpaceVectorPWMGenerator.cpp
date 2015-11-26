/**
 * @author Alexander Entinger, MSc
 * @brief this class allows for the calculation of all variables necessary for generating a space vector pwm signal
 * @file SpaceVectorPWMGenerator.cpp
 */

/* INCLUDES */

#include "SpaceVectorPWMGenerator.h"

/* FUNCTIONS */

/**
 * @brief Constructor
 */
SpaceVectorPWMGenerator::SpaceVectorPWMGenerator() : t1_(0), t2_(0), t1_vect_(VECT_1), t2_vect_(VECT_2), _current_angle_deg(0)
{
	SpaceVectorPWMGenerator::step();
}

/**
 * @brief performs a step
 */
void SpaceVectorPWMGenerator::step()
{
	// find out in which sector we are
	if(_current_angle_deg >= 0 && _current_angle_deg < 60) // sector 1
	{
		t1_vect_ = VECT_1;
		t2_vect_ = VECT_2;
	}
	else if(_current_angle_deg >= 60 && _current_angle_deg < 120) // sector 2
	{
		t1_vect_ = VECT_2;
		t2_vect_ = VECT_3;
	}
	else if(_current_angle_deg >= 120 && _current_angle_deg < 180) // sector 3
	{
		t1_vect_ = VECT_3;
		t2_vect_ = VECT_4;
	}
	else if(_current_angle_deg >= 180 && _current_angle_deg < 240) // sector 4
	{
		t1_vect_ = VECT_4;
		t2_vect_ = VECT_5;
	}
	else if(_current_angle_deg >= 240 && _current_angle_deg < 300) // sector 5
	{
		t1_vect_ = VECT_5;
		t2_vect_ = VECT_6;
	}
	else
	{
		t1_vect_ = VECT_6;
		t2_vect_ = VECT_1;
	}

	t1_ = 255 - (_current_angle_deg % 60) * 255 / 60;
	t2_ = 255 - t1_;

	// increase current angle
	_current_angle_deg += ANGLE_STEP_SIZE;

	// check if there is a overrun over 360° - in that case we need to apply a modulo operation
	if(_current_angle_deg >= 360)
	{
		_current_angle_deg %= 360;
	}
}
