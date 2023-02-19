# ROX-2X_Si5351
Si5351 controller for a 2 m ARDF receiver, based on a Raspberry Pi Pico board

This code controls an Si5351 chip to work as the local oscillator in a direction
finding receiver for the 2 m band (144 MHz). The user interface is a 2x16 alphanumeric
LCD, a rotary encoder and four membrane buttons. It was designed to replace the 
VCO of the ROX-2X ARDF receiver, see http://open-circuit.co.uk/wp/receivers/rox-2x/

The PDF shows the connections around the Pi Pico as well as how the ROX-2X board was 
modified.

The Si5351 library from Adafruit was used, but significantly modified/extended to 
support the needs of this project.

Potentially interesting code includes:

* Extended Adafruit Si5351 library
* Interrupt driven rotary encoder
* An algorithm based on Farey fractions to approximate doubles with fractions
