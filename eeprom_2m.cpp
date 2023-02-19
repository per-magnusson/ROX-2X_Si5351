#include <Arduino.h>
#include "eeprom_2m.h"
#include "rox2x.h"

// Call before use of the EEPROM
void eeprom_init()
{
  // 256 bytes is enough for our purposes
  EEPROM.begin(256);
}


// Stores the valid frequences starting at index 1 up until at most index 9.
void store_frequencies(uint32_t *freqs)
{
  for(int i = 1; i <= 9; i++) {
    if(is_valid_freq(freqs[i])) {
      // Valid frequency, store it
      EEPROM.put((i-1)*sizeof(uint32_t), freqs[i]);
    } else {
      // Invalid frequency, abort writing
      EEPROM.put((i-1)*sizeof(uint32_t), (uint32_t)0xFFFFFFFF); // Mark invalid frequency
      break;
    }
  }
  EEPROM.commit(); // Do the actual writing
}


// Returns the number of used frequencies (the index of the highest valid frequency).
int recall_frequencies(uint32_t *freqs)
{
  uint32_t f;
  int ii;

  for(ii = 1; ii <= 9; ii++) {
    EEPROM.get((ii-1)*sizeof(uint32_t), f);
    if(is_valid_freq(f)) {
      // Valid frequency, use it
      freqs[ii] = f;
    } else {
      // Invalid frequency, abort reading
      freqs[ii] = 0;
      if(ii == 1) {
        freqs[1] = 144710000; // Use this as default so that at least one frequency is valid.
        freqs[2] = 145000000; // #### Debug to test dual frequencies
        ii = 2; // ########
      } else {
        ii--; // Adjust return value
      }
      break;
    }
  }
  return ii;
}
