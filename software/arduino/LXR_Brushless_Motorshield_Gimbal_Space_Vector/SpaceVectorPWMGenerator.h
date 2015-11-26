/**
 * @author Alexander Entinger, MSc
 * @brief this class allows for the calculation of all variables necessary for generating a space vector pwm signal
 * @file SpaceVectorPWMGenerator.h
 */

#ifndef SPACEVECTORPWMGENERATOR_H_
#define SPACEVECTORPWMGENERATOR_H_

/* INCLUDES */

#include <stdint.h>

/* TYPEDEFS */

typedef enum {VECT_1 = 0, VECT_2 = 1, VECT_3 = 2, VECT_4 = 3, VECT_5 = 4, VECT_6 = 5 } E_SPACE_VECTOR_SELECT;

typedef struct
{
  bool W_high;
  bool V_high;
  bool U_high;
} SingleSpaceVector;

/* GLOBAL CONSTANTS */

static SingleSpaceVector const CompleteSpaceVector[6] =
{
  // W_high, V_high, U_high
  {true, true, false},
  {true, false, false},
  {true, false , true},
  {false, false, true},
  {false, true, true},
  {false, true, false}
};

/* PROTOTYPES */

class SpaceVectorPWMGenerator
{

public:

	/**
	 * @brief Constructor
	 */
	SpaceVectorPWMGenerator();

	/**
	 * @brief Destructor
	 */
	~SpaceVectorPWMGenerator() { }

	/**
	 * @brief performs a step
	 */
	void step();

	/**
	 * @brief access the current space vector data
	 */
	inline uint8_t get_t1() const
	{
		return t1_;
	}
	inline uint8_t get_t2() const
	{
		return t2_;
	}
	inline E_SPACE_VECTOR_SELECT get_t1_vect() const
	{
		return t1_vect_;
	}
	inline E_SPACE_VECTOR_SELECT get_t2_vect() const
	{
		return t2_vect_;
	}

private:

	uint8_t t1_, t2_;
	E_SPACE_VECTOR_SELECT t1_vect_, t2_vect_;

	uint16_t _current_angle_deg;

	static uint16_t const ANGLE_STEP_SIZE = 5;

};

#endif /* SPACEVECTORPWMGENERATOR_H_ */
