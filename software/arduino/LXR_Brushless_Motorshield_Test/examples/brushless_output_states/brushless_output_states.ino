/**
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this sketch utilizes the Brushless_Test library to test the brushless motor shield
 * @file brushless_output_states.ino
 * @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */

#include <LXR_Brushless_Motorshield_Test.h>

void setup() {
  LXR_Brushless_Motorshield_Test::begin();
  Serial.begin(115200);
}

static E_OUTPUT_STATE state_u = O_LOW;
static E_OUTPUT_STATE state_v = O_LOW;
static E_OUTPUT_STATE state_w = O_LOW;

void loop() {
  if(Serial.available()) {
    switch(Serial.read()) {
    case 'u': 
      {
        LXR_Brushless_Motorshield_Test::set_state_U(state_u);
        Serial.print("Current State U: ");
        print_output_state(state_u);
        state_u = incr_output_state(state_u);
      } 
      break;
    case 'v': 
      {
        LXR_Brushless_Motorshield_Test::set_state_V(state_v);
        Serial.print("Current State V: ");
        print_output_state(state_v);
        state_v = incr_output_state(state_v);
      } 
      break; 
    case 'w': 
      {
        LXR_Brushless_Motorshield_Test::set_state_W(state_w);
        Serial.print("Current State W: ");
        print_output_state(state_w);
        state_w = incr_output_state(state_w);
      } 
      break;
    default: 
      break;
    }
  } 
}

/**
 * @brief this function increments the output state
 */
E_OUTPUT_STATE incr_output_state(E_OUTPUT_STATE const state) {
  if(state == O_LOW) return O_HIGH;
  else if(state == O_HIGH) return O_HIGH_IMPEDANCE;
  else if(state == O_HIGH_IMPEDANCE) return O_LOW;
  else return O_LOW;
}

/**
 * @brief prints the current output state on the serial line
 */
void print_output_state(E_OUTPUT_STATE const state) {
  switch(state) {
  case O_LOW: 
    Serial.println("LOW"); 
    break; 
  case O_HIGH: 
    Serial.println("HIGH"); 
    break;
  case O_HIGH_IMPEDANCE: 
    Serial.println("HIGH_IMPEDANCE"); 
    break;
  default: 
    Serial.println("ERROR"); 
    break;
  }
}

