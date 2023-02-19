/* Interrupt driven rotary encoder routines.
   Target: Teensy 3.1, 4.0 etc
   
   Written by Per Magnusson, http://www.axotron.se
   v 1.0 2015-11-15
   v 1.1 2021-01-30 (object oriented rewrite)
   v 1.2 2023-01-01 (rewrite to avoid "static initialization order fiasco" 
                     and require a call to .begin())
   This program is public domain.
*/

#include "encoder2.h"


Encoder2::Encoder2(int pin_a_a, int pin_b_a, void isr_a(void), void isr_b(void)):
  pin_a(pin_a_a), pin_b(pin_b_a), _isr_a(isr_a), _isr_b(isr_b)
{
  rot_count = 0;
  rot_a_val = 1;
  rot_b_val = 1;
  rot_state = 0;
}

void Encoder2::begin(void)
{
  pinMode(pin_a, INPUT_PULLUP);
  pinMode(pin_b, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_a), _isr_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_b), _isr_b, CHANGE);
}

int Encoder2::read(void)
{
  volatile int n;
   
  noInterrupts(); // Turn off interrupts
  n = rot_count;
  interrupts();
  return n; 
}


void Encoder2::write(int n)
{
  noInterrupts(); // Turn off interrupts
  rot_count = n;
  interrupts();
}


// Interrupt routines
void Encoder2::isr_rot_a_change()
{
  if(digitalRead(pin_a)) {
    rot_a_val = 1;
    update_rot();
  } else {
    rot_a_val = 0;
    update_rot();
  }
}


void Encoder2::isr_rot_b_change() 
{
  if(digitalRead(pin_b)) {
    rot_b_val = 1;
    update_rot();
  } else {
    rot_b_val = 0;
    update_rot();
  }
}


// Update rotary encoder accumulator. 
// This function is called by the interrupt routines.
void Encoder2::update_rot() 
{
  // Increment rot_count if going CW, decrement it if going CCW.
  // Do not increment anything if it was just a glitch.
  switch(rot_state) {
    case 0: // Idle state, look for direction
      if(!rot_b_val) {
        rot_state = 1;  // CW 1
      }
      if(!rot_a_val) {
        rot_state = 11; // CCW 1
      }
      break;
    case 1: // CW, wait for A low while B is low
      if(!rot_b_val) {
        if(!rot_a_val) {
          rot_count++;
          rot_state = 2; // CW 2
        }
      } else {
        if(rot_a_val) {
          // It was just a glitch on B, go back to start
          rot_state = 0;
        }
      }
      break;
    case 2: // CW, wait for B high
      if(rot_b_val) {
        rot_state = 3; // CW 3
      }
      break;
    case 3: // CW, wait for A high
      if(rot_a_val) {
        rot_state = 0; // back to idle (detent) state
      }
      break;
    case 11: // CCW, wait for B low while A is low
      if(!rot_a_val) {
        if(!rot_b_val) {
          rot_count--;
          rot_state = 12; // CCW 2
        }
      } else {
        if(rot_b_val) {
          // It was just a glitch on A, go back to start
          rot_state = 0;
        }
      }
      break;
    case 12: // CCW, wait for A high
      if(rot_a_val) {
        rot_state = 13; // CCW 3
      }
      break;
    case 13: // CCW, wait for B high
      if(rot_b_val) {
        rot_state = 0; // back to idle (detent) state
      }
      break;
  }
}

