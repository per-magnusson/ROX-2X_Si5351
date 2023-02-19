#ifndef Encoder2_h_
#define Encoder2_h_

/* Interrupt driven rotary encoder routines.
   Target: Teensy 3.1, 4.0 etc
   
   Written by Per Magnusson, http://www.axotron.se
   v 1.0 2015-11-15
   v 1.1 2021-01-30 (object oriented rewrite)
   v 1.2 2023-01-01 (rewrite to avoid "static initialization order fiasco" 
                     and require a call to .begin())
   This program is public domain.
*/

#include <Arduino.h>

class Encoder2 
{
public:
  // C++ does not handle pointers to (non-static) methods in an instance
  // of a class like other function pointers, so we need to work 
  // around this by suppling pointers to outside functions that 
  // just call the isr_rot_X_change() methods of the appropriate
  // instance.
  Encoder2(int pin_a_a, int pin_b_a, void isr_a(void), void isr_b(void));
  void begin(void);
  int read(void);
  void write(int n);
  void isr_rot_a_change(void);
  void isr_rot_b_change(void);
private:
  int rot_count;
  int rot_a_val;
  int rot_b_val;
  int rot_state;
  int pin_a;
  int pin_b;
  void (*_isr_a)(void); // Temporary storage between constructor and begin()
  void (*_isr_b)(void);

  void update_rot(void);
};

#endif
