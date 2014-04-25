/**
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this file implements an arduino library to interface the LXRobotics Brushless Motorshield
 * @file LXR_Brushless_Motorshield.c
 * @license Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0) ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */

#include "LXR_Brushless_Motorshield.h"

/* C IMPLEMENTATION */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdbool.h>

// enabling this defines puts the signal of the comparator isr on the digital pin D9 (PB1) ... useful for triggering the oszillocop
//#define BRUSHLESS_DEBUG_OSZI

/* HARDWARE ABSTRACTION LAYER */
#ifdef BRUSHLESS_DEBUG_OSZI
// analog converter interrupt pin for triggering the oszi = D9 = PB1
#define AC_INT_DDR		(DDRB)
#define AC_INT_PORT		(PORTB)
#define AC_INT			(1<<1)
#endif
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
// MUX settings for connecting the bemf pin to the AIN1 input of the analog comparator
// BEMF_W = A5 = ADC5
#define MUX_BEMF_W_TO_AIN1	ADMUX &= 0xF0; ADMUX |= (1<<MUX2) | (1<<MUX0);
// BEMF_V = A4 = ADC4
#define MUX_BEMF_V_TO_AIN1	ADMUX &= 0xF0; ADMUX |= (1<<MUX2);
// BEMF_U = A3 = ADC3
#define MUX_BEMF_U_TO_AIN1	ADMUX &= 0xF0; ADMUX |= (1<<MUX1) | (1<<MUX0);

/* GLOBAL VARIABLES */
static volatile int8_t m_state = 0;
static volatile uint8_t m_speed = 0;
static volatile bool m_startup_complete = false;
static volatile E_DIRECTION m_dir = FORWARD;

/* PROTOTYPE SECTION */
void C_set_pins_according_to_state();

/* PROGRAM SECTION */

/**
 * @brief initializes the brushless motor control
 */
void C_init_brushless_motor_control() {
#ifdef BRUSHLESS_DEBUG_OSZI
  AC_INT_PORT &= ~AC_INT;
  AC_INT_DDR |= AC_INT;
#endif
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

  // set the pins according to the current state
  C_set_pins_according_to_state();

  // init the analog comparator
  // turn off adc
  ADCSRA &= ~(1<<ADEN);
  // turn off the digital input buffer on the pin AIN0 (PD6, SUM Signal)
  DIDR1 |= (1<<AIN0D);
  // turn off the digital input buffer on the pins ADC5, ADC4, ADC3
  DIDR0 |= (1<<ADC5D) | (1<<ADC4D) | (1<<ADC3D);
  // use multiplexer for negative comparator input
  ADCSRB |= (1<<ACME);
  // analog comparator is enabled
  ACSR &= ~(1<<ACD);
  // trigger interrupt on falling edge of comparator
  ACSR |= (1<<ACIS1);
  // enable analog comparator interrupt
  ACSR |= (1<<ACIE);

  // init timer 2 (for pwm generation) - output is activated on timer overflow, deactivated at compare match
  // clear TCCR2A
  TCCR2A = 0x00;
  // clear TCNT2
  TCNT2 = 0;
  // enable timer 2 overflow and compare match a interrupt
  TIMSK2 |= (1<<TOIE2) | (1<<OCIE2A);
  // set compare match register to zero
  OCR2A = 0;
  // prescaler = 1, 1 Timerstep = 500 ns, 255 Timersteps = 127.5 usm, fPWM = 7,8 kHz
  TCCR2B = (1<<CS20);

  // init watchdog as timeout timer
  WDTCSR = (1<<WDCE) | (1<<WDIE) | (1<<WDP2) | (1<<WDP1);
}

/** 
 * @brief sets the speed for the brushless motor
 * @param speed
 */
void C_set_speed(uint8_t const speed) {
  m_speed = speed;
  OCR2A = m_speed;
  if(m_speed == 0) TCCR2B &= ~(1<<CS20);
  else TCCR2B |= (1<<CS20); 
}

/** 
 * @brief this functions sets the output and inputs according to the motor state
 */
void C_set_pins_according_to_state() {
  switch(m_state) {
  case 0: 
    { 
      if(m_dir == FORWARD) {
        // U = PWM, V = Float, W = GND
        INH_U_PORT |= INH_U;	
        IN_U_PORT |= IN_U;
        INH_V_PORT &= ~INH_V;	
        MUX_BEMF_V_TO_AIN1;
      } else {
        // U = Float, V = PWM, W = GND
        INH_U_PORT &= ~INH_U;	
        MUX_BEMF_U_TO_AIN1;
        INH_V_PORT |= INH_V;	
        IN_V_PORT |= IN_V;
      }
      INH_W_PORT |= INH_W;	
      IN_W_PORT &= ~IN_W;
    } 
    break;
  case 1: 
    { 
      if(m_dir == FORWARD) {
        // U = Float, V = PWM, W = GND
        INH_U_PORT &= ~INH_U;	
        MUX_BEMF_U_TO_AIN1;
        INH_V_PORT |= INH_V;	
        IN_V_PORT |= IN_V;
      } else {
         // U = PWM, V = Float, W = GND
        INH_U_PORT |= INH_U;	
        IN_U_PORT |= IN_U;
        INH_V_PORT &= ~INH_V;	
        MUX_BEMF_V_TO_AIN1;
      }
      INH_W_PORT |= INH_W;	
      IN_W_PORT &= ~IN_W;
    } 
    break;
  case 2: 
    { 
      if(m_dir == FORWARD) {
        // U = GND, V = PWM, W = Float
        INH_U_PORT |= INH_U;	
        IN_U_PORT &= ~IN_U;
        INH_V_PORT |= INH_V;	
        IN_V_PORT |= IN_V;
      } else {
        // U = PWM, V = GND, W = Float
        INH_U_PORT |= INH_U;	
        IN_U_PORT |= IN_U;
        INH_V_PORT |= INH_V;	
        IN_V_PORT &= ~IN_V;
      }
      INH_W_PORT &= ~INH_W;	
      MUX_BEMF_W_TO_AIN1;
    } 
    break;
  case 3: 
    { 
      if(m_dir == FORWARD) {
        // U = GND, V = Float , W =  PWM
        INH_U_PORT |= INH_U;	
        IN_U_PORT &= ~IN_U;
        INH_V_PORT &= ~INH_V;	
        MUX_BEMF_V_TO_AIN1;
      } else {
        // U = Float, V = GND , W =  PWM
        INH_U_PORT &= ~INH_U;	
        MUX_BEMF_U_TO_AIN1;
        INH_V_PORT |= INH_V;	
        IN_V_PORT &= ~IN_V;
      }
      INH_W_PORT |= INH_W;	
      IN_W_PORT |= IN_W;
    } 
    break;
  case 4: 
    { 
      if(m_dir == FORWARD) {
        // U = Float, V = GND, W = PWM
        INH_U_PORT &= ~INH_U;	
        MUX_BEMF_U_TO_AIN1;
        INH_V_PORT |= INH_V;	
        IN_V_PORT &= ~IN_V;
      } else {
        // U = GND, V = Float, W = PWM
        INH_U_PORT |= INH_U;	
        IN_U_PORT &= ~IN_U;
        INH_V_PORT &= ~INH_V;	
        MUX_BEMF_V_TO_AIN1;
      }
      INH_W_PORT |= INH_W;	
      IN_W_PORT |= IN_W;
    } 
    break;
  case 5: 
    { 
      if(m_dir == FORWARD) {
        // U = PWM, V = GND, W = Float
        INH_U_PORT |= INH_U;	
        IN_U_PORT |= IN_U;
        INH_V_PORT |= INH_V;	
        IN_V_PORT &= ~IN_V;
      } else {
        // U = GND, V = PWM, W = Float
        INH_U_PORT |= INH_U;	
        IN_U_PORT &= ~IN_U;
        INH_V_PORT |= INH_V;	
        IN_V_PORT |= IN_V;
      }
      INH_W_PORT &= ~INH_W;	
      MUX_BEMF_W_TO_AIN1;
    } 
    break;
  default: 
    {

    } 
    break;
  }
}

/** 
 * @brief interrupt service routine for analog comparator interrupt
 */
ISR(ANALOG_COMP_vect) {
  static uint8_t startup_isr_cnt = 0;
#ifdef BRUSHLESS_DEBUG_OSZI
  AC_INT_PORT |= AC_INT;
#endif
  if(m_startup_complete) {	
    // switch to the next state
    m_state++; 
    if(m_state == 6) m_state = 0;
    // and set the pins according to the new state
    C_set_pins_according_to_state();
    // toogle on wich egde the ac is triggering
    ACSR ^= (1<<ACIS0);
  } 
  else {
    startup_isr_cnt++;
    if(startup_isr_cnt > 25) {
      m_startup_complete = true;
      startup_isr_cnt = 0;
      m_state = 0;
    }			
  }		
  // reset timer 2
  TCNT2 = 0;
  // reset watchdog here, if watchdog is reset here regularly, it shows that the motor is always running
  wdt_reset();
}

/** 
 * @brief interrupt service routine for timer 2 overflow interrupt
 */
ISR(TIMER2_OVF_vect) {
#ifdef BRUSHLESS_DEBUG_OSZI
  AC_INT_PORT &= ~AC_INT;
#endif

  // activate current pwm pin
  switch(m_state) {
  case 0: 
    IN_U_PORT |= IN_U; 
    break; // U = PWM
  case 1: 
    IN_V_PORT |= IN_V; 
    break; // V = PWM
  case 2: 
    IN_V_PORT |= IN_V; 
    break; // V = PWM
  case 3: 
    IN_W_PORT |= IN_W; 
    break; // W = PWM
  case 4: 
    IN_W_PORT |= IN_W; 
    break; // W = PWM
  case 5: 
    IN_U_PORT |= IN_U; 
    break; // U = PWM
  default: 
    break;
  }
}

/** 
 * @brief interrupt service routine for timer 2 compare match a interrupt
 */
ISR(TIMER2_COMPA_vect) {
  // deactivate current pwm pin
  switch(m_state) {
  case 0: 
    IN_U_PORT &= ~IN_U; 
    break; // U = PWM
  case 1: 
    IN_V_PORT &= ~IN_V; 
    break; // V = PWM
  case 2: 
    IN_V_PORT &= ~IN_V; 
    break; // V = PWM
  case 3: 
    IN_W_PORT &= ~IN_W; 
    break; // W = PWM
  case 4: 
    IN_W_PORT &= ~IN_W; 
    break; // W = PWM
  case 5: 
    IN_U_PORT &= ~IN_U; 
    break; // U = PWM
  default: 
    break;
  }
}

/** 
 * @brief watchdog isr: if this occurs the motor did either never start or was forcefully stopped
 */
ISR(WDT_vect) {
  wdt_reset();
  WDTCSR |= (1<<WDCE) | (1<<WDIE); // reenable interrupt to prevent system reset
  if(m_speed > 0) { 
    sei();
    m_startup_complete = false;
    while(!m_startup_complete) {
      // switch to the next state
      m_state++; 
      if(m_state == 6) m_state = 0;
      // and set the pins according to the new state
      C_set_pins_according_to_state();
      // wait a little
      _delay_ms(10);
    }
  }		
}

/* ARDUINO INTERFACE */

/**
 * @brief initialize the brushless shield
 */
void LXR_Brushless_Motorshield::begin() {
  C_init_brushless_motor_control();
}

/**
 * @brief sets the speed of the brushless motor
 * @param speed value between 0 and 255, where 0 is stop and 255 is full speed
 */
void LXR_Brushless_Motorshield::set_speed(uint8_t const speed) { 
  C_set_speed(speed);
}

/**
 * @brief returns the current speed
 */
uint8_t LXR_Brushless_Motorshield::get_speed() {
  return m_speed; 
}

/**
 * @brief sets the direction of the brushless motor
 */
void LXR_Brushless_Motorshield::set_direction(E_DIRECTION const dir) {
  m_dir = dir;
}
  
/**
 * @brief sets the direction of the brushless motor
 */
E_DIRECTION LXR_Brushless_Motorshield::get_direction() {
  return m_dir;
}
