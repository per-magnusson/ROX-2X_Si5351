#ifndef eeprom_2m_h_
#define eeprom_2m_h_

#include <EEPROM.h>

void eeprom_init(); // Call before use of the EEPROM
void store_frequencies(uint32_t *freqs); // Stores the valid frequences starting at index 1
int recall_frequencies(uint32_t *freqs); // Returns the number of used frequencies, at least 1

#endif