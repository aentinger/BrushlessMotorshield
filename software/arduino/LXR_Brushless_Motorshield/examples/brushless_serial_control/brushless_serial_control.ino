/**
 * @author Marco Schilling, MSc / TU Ilmenau
 * @brief this file demonstrates how to use the brushless motor
 * @file brushless_serial_control.ino
 */

/* INCLUDES */

#include "LXR_Brushless_Motorshield.h"

/* GLOBAL CONSTANTS */

static int const LED = LED;

/* FUNCTIONS */

void setup() 
{
  //Setup and configuration of the control algorithm
  LXR_Brushless_Motorshield::begin();
  LXR_Brushless_Motorshield::set_direction(FORWARD);
  //LXR_Brushless_Motorshield::set_direction(BACKWARD);
  LXR_Brushless_Motorshield::set_speed(0);

  //Serial communication setup
  Serial.begin(115200);

  //Send intialisation data
  Serial.println("Speed = 0");

  // On-Board LED Pin -> Output
  pinMode(LED, OUTPUT);
}

void loop() 
{
  //Input speed value via terminal
  if (Serial.available() > 0) 
  {
    int const speed = Serial.parseInt();
    Serial.print("Speed Eingabe = ");
    Serial.println(speed);

    LXR_Brushless_Motorshield::set_speed(speed);
    
    Serial.println();
    Serial.print("Speed gesetzt = ");
    Serial.println(LXR_Brushless_Motorshield::get_speed());  
  } //End if

  //Show system state via LED: LED on -> startup - mode
  if (!LXR_Brushless_Motorshield::is_startup_complete())
  {
    digitalWrite(LED, HIGH);
  }
  else
  {
    digitalWrite(LED, LOW);
  } 
}

