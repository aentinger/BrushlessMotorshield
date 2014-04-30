/*
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this file implements the library to control a brushless gimbal motor with the LXRobotics Brushless Motorshield
 * @file LXR_Brushless_Motorshield_Gimbal.cpp
 * @license Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0) ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */ 

#include "LXR_Brushless_Motorshield_Gimbal.h"

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

/* TYPEDEFS */
typedef enum {U = 0, V = 1, W = 2} E_PHASE_SELECT;
typedef struct {
	uint8_t ocr2a;
	E_PHASE_SELECT phase;
} s_phase_data;
typedef struct {
	s_phase_data phase_data[3];
	uint8_t phase_cnt;
} s_pwm_data;

/* GLOBAL VARIABLES */
static volatile E_DIRECTION m_direction = FORWARD;
static volatile uint8_t m_current_step_U = 0;
static volatile uint8_t m_current_step_V = 14;
static volatile uint8_t m_current_step_W = 28;
static volatile s_pwm_data m_pwm_data;

/* GLOBAL CONSTANTS */
static const uint8_t NUMBER_OF_STEPS = 42;
static const uint8_t SINUS[NUMBER_OF_STEPS] = {128,147,166,184,200,215,228,239,247,253,255,255,253,247,239,228,215,200,184,166,147,128,109,90,72,56,41,28,17,9,3,0,0,3,9,17,28,41,56,72,90,109};	

/* PROTOTYPES */
uint8_t increase_step(uint8_t const step);
void sort(s_phase_data *data, uint8_t const length);
void swap(s_phase_data *a, s_phase_data *b);


/* FUNCTIONS */

/** 
 * @brief initializes this library
 */	
void LXR_Brushless_Motorshield_Gimbal::begin() {
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
	  
	  // set up all params for the pwm generation my making one step
	  LXR_Brushless_Motorshield_Gimbal::step();
	 
	  // set up timer 2
	  // reset timer
	  TCNT2 = 0;
	  // enable interrupts compare match A, timer overflow
	  TIMSK2 = (1<<OCIE2A) | (1<<TOIE2);
	  // run the timer
	  // prescaler = 8
	  // fTimer = fClk / 8 = 16 MHz / 8 = 2 MHz
	  // tTimerStep = 1 / 2 MHz = 500 ns
	  // tTimerCycle = 256 * tTimerStep = 128 us
	  // fTimerCycle = 1 / tTimerCycle = 7.8125 kHz
	  TCCR2B = (1<<CS21);
	  
	  // enable the bridges
	  ENABLE_U();
	  ENABLE_V();
	  ENABLE_W();
}
	
/**
 * @brief performs one step in the desired direction
 */
void LXR_Brushless_Motorshield_Gimbal::step() {
	// update step	
	m_current_step_U = increase_step(m_current_step_U);
	m_current_step_V = increase_step(m_current_step_V);
	m_current_step_W = increase_step(m_current_step_W);
	
	// setup phase data structs with the ocr values and phase
	s_phase_data phase_u, phase_v, phase_w;
	phase_u.ocr2a = SINUS[m_current_step_U];
	phase_u.phase = U;
	phase_v.ocr2a = SINUS[m_current_step_V];
	phase_v.phase = V;
	phase_w.ocr2a = SINUS[m_current_step_W];
	phase_w.phase = W;

        if(m_direction == BACKWARD) {
      	  phase_u.phase = V; 
      	  phase_v.phase = U; 
        }
	
	// create an array and sort it ascending
	s_phase_data phases[3] = {phase_u, phase_v, phase_w};
	sort(phases, 3);
	
	// update pwm data struct
	uint8_t p = 0; for(; p<3; p++) {
		m_pwm_data.phase_data[p].ocr2a = phases[p].ocr2a;
		m_pwm_data.phase_data[p].phase = phases[p].phase;
	}
}
	
/**
 * @brief sets the direction for the brushless motor
 */
void LXR_Brushless_Motorshield_Gimbal::set_direction(E_DIRECTION const dir) {
	m_direction = dir;
}

/**
 * @brief increases by one step
 */
uint8_t increase_step(uint8_t const step) {
	if(step == (NUMBER_OF_STEPS - 1)) return 0;
	else return (step + 1);
}

/** 
 * @brief sorts the phases ascending by their ocr values
 */
void sort(s_phase_data *data, uint8_t const length) {
	bool elems_swapped = false;
	do {
		elems_swapped = false;
		uint8_t i = 1; for(; i<length; i++) {
			if(data[i-1].ocr2a > data[i].ocr2a) {
				swap(&data[i-1], &data[i]);
				elems_swapped = true;
			}
		}
		
	} while(elems_swapped);
}

/** 
 * @brief swaps two structs
 */
void swap(s_phase_data *a, s_phase_data *b) {
	s_phase_data tmp = *a;
	*a = *b;
	*b = tmp;
}

/** 
 * @brief interrupt service routine for timer 2 overflow vector
 */
ISR(TIMER2_OVF_vect) {
	// turn on all phases
	if(SINUS[m_current_step_U] > 0) SET_U_HIGH();
	if(SINUS[m_current_step_V] > 0) SET_V_HIGH();
	if(SINUS[m_current_step_W] > 0) SET_W_HIGH();
	// update phase counter
	m_pwm_data.phase_cnt = 0;
	// update reg
	OCR2A = m_pwm_data.phase_data[m_pwm_data.phase_cnt].ocr2a;
}

/** 
 * @brief interrupt service routine for timer 2 compare match A vector
 */
ISR(TIMER2_COMPA_vect) {
	// turn off corresponding output
	switch(m_pwm_data.phase_data[m_pwm_data.phase_cnt].phase) {
		case U: SET_U_LOW(); break;
		case V: SET_V_LOW(); break;
		case W: SET_W_LOW(); break;
		default: break;
	}
	// update phase counter
	m_pwm_data.phase_cnt++;
	// update register
	OCR2A = m_pwm_data.phase_data[m_pwm_data.phase_cnt].ocr2a;
}
