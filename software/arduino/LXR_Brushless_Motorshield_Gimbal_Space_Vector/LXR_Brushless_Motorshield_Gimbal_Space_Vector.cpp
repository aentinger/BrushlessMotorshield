/*
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this file implements the library to control a brushless gimbal motor with the LXRobotics Brushless Motorshield
 * @file LXR_Brushless_Motorshield_Gimbal_Space_Vector.cpp
 * @license Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0) ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */ 

#include "LXR_Brushless_Motorshield_Gimbal_Space_Vector.h"

#include <stdint.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/interrupt.h>

/* DEFINES */

// IN_W = D8 = PB0
#define IN_W_DDR		(DDRB)
#define IN_W_PORT		(PORTB)
#define IN_W			(1<<0)
// INH_W = D7 = PD7
#define INH_W_DDR		(DDRD)
#define INH_W_PORT		(PORTD)
#define INH_W			(1<<7)
// IN_V = D4 = PD4
#define IN_V_DDR		(DDRD)
#define IN_V_PORT		(PORTD)
#define IN_V			(1<<4)
// INH_V = D5 = PD5
#define INH_V_DDR		(DDRD)
#define INH_V_PORT		(PORTD)
#define INH_V			(1<<5)
// IN_U = D2 = PD2
#define IN_U_DDR		(DDRD)
#define IN_U_PORT		(PORTD)
#define IN_U			(1<<2)
// INH_U = D3 = PD3
#define INH_U_DDR		(DDRD)
#define INH_U_PORT		(PORTD)
#define INH_U			(1<<3)

/* MACROS */

#define ENABLE_U() do { INH_U_PORT |= INH_U; } while (0)
#define ENABLE_V() do { INH_V_PORT |= INH_V; } while (0)
#define ENABLE_W() do { INH_W_PORT |= INH_W; } while (0)
#define SET_U_HIGH() do { IN_U_PORT |= IN_U; } while (0)
#define SET_V_HIGH() do { IN_V_PORT |= IN_V; } while (0)
#define SET_W_HIGH() do { IN_W_PORT |= IN_W; } while (0)
#define SET_U_LOW() do { IN_U_PORT &= ~IN_U; } while (0)
#define SET_V_LOW() do { IN_V_PORT &= ~IN_V; } while (0)
#define SET_W_LOW() do { IN_W_PORT &= ~IN_W; } while (0)

/* GLOBAL VARIABLES */

static volatile E_DIRECTION dir = FORWARD;
static volatile E_SPACE_VECTOR_SELECT vect_1 = VECT_1, vect_2 = VECT_2;

/* PROTOTYPES */

void set_space_vector(E_DIRECTION const dir, E_SPACE_VECTOR_SELECT const desired_space_vector);

/* FUNCTIONS */

/** 
 * @brief Constructor
 */
LXR_Brushless_Motorshield_Gimbal_Space_Vector::LXR_Brushless_Motorshield_Gimbal_Space_Vector()
{
  
}

/** 
 * @brief initializes this library
 */	
void LXR_Brushless_Motorshield_Gimbal_Space_Vector::begin() 
{
	  // set all IN_x ports to 0
	  IN_U_PORT &= ~IN_U;
	  IN_V_PORT &= ~IN_V;
	  IN_W_PORT &= ~IN_W;
	  // set all IN_x ports to outputs
	  IN_U_DDR |= IN_U;
	  IN_V_DDR |= IN_V;
	  IN_W_DDR |= IN_W;

	  // set all INH_X ports to 0
    INH_U_PORT &= ~INH_U;
	  INH_V_PORT &= ~INH_V;
	  INH_W_PORT &= ~INH_W;
	  // set all INH_x ports to outputs
	  INH_U_DDR |= INH_U;
	  INH_V_DDR |= INH_V;
	  INH_W_DDR |= INH_W;

    // setup all variables
    this->step();

	  // set up timer 2
    // set to normal operation mode
    TCCR2A = 0;
    // reset timer
    TCNT2 = 0;
    // run the timer
    // prescaler = 8
    // fTimer = fClk / 8 = 16 MHz / 8 = 2 MHz
    // tTimerStep = 1 / 2 MHz = 500 ns
    // tTimerCycle = 256 * tTimerStep = 128 us
    // fTimerCycle = 1 / tTimerCycle = 7.8125 kHz
    TCCR2B = (1<<CS21);
	  // enable interrupts for timer 2 overflow
	  TIMSK2 = (1<<OCIE2A) | (1<<TOIE2);
	  
	  // enable the bridges
	  ENABLE_U();
	  ENABLE_V();
	  ENABLE_W();
}
	
/**
 * @brief performs one step in the desired direction
 */
void LXR_Brushless_Motorshield_Gimbal_Space_Vector::step() 
{
  svpwm_gen_.step();
  
  OCR2A = svpwm_gen_.get_t1();
  
  vect_1 = svpwm_gen_.get_t1_vect();
  
  vect_2 = svpwm_gen_.get_t2_vect();
}

/**
 * @brief sets the direction for the brushless motor
 */
void LXR_Brushless_Motorshield_Gimbal_Space_Vector::set_direction(E_DIRECTION const direction)
{
  dir = direction;  
}

/** 
 * @brief interrupt service routine for timer 2 overflow vector
 */
ISR(TIMER2_OVF_vect) 
{
  // turn on vector for t1
  set_space_vector(dir, vect_1);
}

/** 
 * @brief interrupt service routine for timer 2 compare match A vector
 */
ISR(TIMER2_COMPA_vect) 
{
    // turn on vector for t2
    set_space_vector(dir, vect_2);
}
	
void set_space_vector(E_DIRECTION const dir, E_SPACE_VECTOR_SELECT const desired_space_vector)
{
  if(CompleteSpaceVector[desired_space_vector].U_high)
  {
    if(dir == FORWARD) SET_U_HIGH();
    else SET_V_HIGH();
  }
  else
  {
     if(dir == FORWARD) SET_U_LOW();
     else SET_V_LOW();
  }
  
  if(CompleteSpaceVector[desired_space_vector].V_high)
  {
    if(dir == FORWARD) SET_V_HIGH();
    else SET_U_HIGH();
  }
  else
  {
     if(dir == FORWARD) SET_V_LOW();
     else SET_U_LOW();
  }
  
  if(CompleteSpaceVector[desired_space_vector].W_high)
  {
    SET_W_HIGH();
  }
  else
  {
     SET_W_LOW();
  }
}
