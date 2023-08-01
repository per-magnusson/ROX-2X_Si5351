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

The normal view during an ARDF competition is that the top row of the display shows the battery status and 
a bar with the signal strength (RSSI). The length of this bar is dependent on the 
analog gain setting but could potentially be used to compare the signal along different 
bearings. Normally one would however just listen to the audio RSSI while sweeping the 
antenna in different directions.

The second row shows the tuned frequency and which of the preprogrammed frequencies is
currently selected. 

If neither the encoder, nor any of the four membrane keys have been used for the last 
10 seconds, they all become locked, except for key 1 which unlocks them. When they are 
unlocked, they have the following functions:

* 1, edit a more significant digit in the frequency.
* 2, edit a less significant digit in the frequency.
* 3, step to the next stored frequency.
* 4, hold for a second to enter the "menu". "M" is shown in the lower right corner during this waiting period.
* Encoder, increment/decrement the currently active digit of the frequency.

When in the "menu", the first view allows editing of the stored frequencies. Key 1 changes which 
digit is edited. Key 2 jumps to the next frequency (up to 9 stored frequencies are supported). 
Key 3 stores the frequencies up to and including the currently selected frequency. 
Key 4 aborts this step and goes to the next menu screen without modifying the stored frequencies.

The second menu screen just shows the battery voltage. Pressing 4 moves on to the next screen.

The third menu screen can activate a function to characterize the pass band response 
(including the image reponse) of the receiver. A USB cable where a serial terminal looks at the printed 
output from the reciever shall be used to collect the resulting frequency/amplitude value pairs
which can then be plotted in e.g. Excel. An internal signal generator is used to create the 
test RF signal. Ideally, no antenna should be connected during this test. The result depends on the
gain setting. Press 1 to run this test or 4 to skip it.

The fourth menu item sweeps the tuning across the band and reports the RSSI at many frequencies 
as printouts on the USB serial port. This test could be used to look for internal disturbances from
the receiver itself, particularly if no antenna is connected during the test. The result depends on the
gain setting. Press 1 to run this test or 4 to skip it.

After the fourth menu screen, we are back to the main view used during a competition. So, to quickly go through the menus
if one accidentally enters it, key 4 can be pressed a couple of times to get back to the standard view.
