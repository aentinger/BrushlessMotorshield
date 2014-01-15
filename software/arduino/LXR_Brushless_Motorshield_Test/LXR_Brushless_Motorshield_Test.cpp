/**
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this module implements a test library for the brushless motor shield
 * @file LXR_Brushless_Motorshield_Test.cpp
 * @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */
 
#include <LXR_Brushless_Motorshield_Test.h>

static int const IN_U = 2; // D2
static int const INH_U = 3; // D3
static int const IN_V = 4; // D4
static int const INH_V = 5; // D5
static int const IN_W = 8; // D8
static int const INH_W = 7; // D7
 
/**
 * @brief initializes the lxr brushless motorshield test library
 */ 
void LXR_Brushless_Motorshield_Test::begin() {
	// set all pins as output
	pinMode(IN_U, OUTPUT);
	pinMode(INH_U, OUTPUT);
	pinMode(IN_V, OUTPUT);
	pinMode(INH_V, OUTPUT);
	pinMode(IN_W, OUTPUT);
	pinMode(INH_V, OUTPUT);
	
	// set to high z (set INH_x to LOW)
	digitalWrite(INH_U, LOW);
	digitalWrite(INH_V, LOW);
	digitalWrite(INH_W, LOW);
}
	
/**
 * @brief set the output state for U
 * @param state output state to set
 */
void LXR_Brushless_Motorshield_Test::set_state_U(E_OUTPUT_STATE const state) {
	switch(state) {
	case O_LOW: {
		digitalWrite(IN_U, LOW);
		digitalWrite(INH_U, HIGH);
	} break;
	case O_HIGH: {
		digitalWrite(IN_U, HIGH);
		digitalWrite(INH_U, HIGH);
	} break;
	case O_HIGH_IMPEDANCE: {
		digitalWrite(INH_U, LOW);
	} break;
	default: break;
	}
}
	
/**
 * @brief set the output state for V
 * @param state output state to set
 */
void LXR_Brushless_Motorshield_Test::set_state_V(E_OUTPUT_STATE const state) {
	switch(state) {
	case O_LOW: {
		digitalWrite(IN_V, LOW);
		digitalWrite(INH_V, HIGH);
	} break;
	case O_HIGH: {
		digitalWrite(IN_V, HIGH);
		digitalWrite(INH_V, HIGH);
	} break;
	case O_HIGH_IMPEDANCE: {
		digitalWrite(INH_V, LOW);
	} break;
	default: break;
	}
}
	
/**
 * @brief set the output state for W
 * @param state output state to set
 */
void LXR_Brushless_Motorshield_Test::set_state_W(E_OUTPUT_STATE const state) {
	switch(state) {
	case O_LOW: {
		digitalWrite(IN_W, LOW);
		digitalWrite(INH_W, HIGH);
	} break;
	case O_HIGH: {
		digitalWrite(IN_W, HIGH);
		digitalWrite(INH_W, HIGH);
	} break;
	case O_HIGH_IMPEDANCE: {
		digitalWrite(INH_W, LOW);
	} break;
	default: break;
	}
}