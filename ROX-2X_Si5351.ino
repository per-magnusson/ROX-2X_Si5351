// 2 m ARDF receiver local oscillator controller for ROX-2X
// based on an Raspberry Pi Pico board.
// Set the CPU clock frequency to 50 MHz in the Arduino IDE (Tools->CPU speed)
// to not have it near the LO and to save current.
// 2023-01-08 Per Magnusson
// This code is under the MIT license.

#include <LiquidCrystal.h>
#include <Bounce2.h>
#include <pico/stdlib.h>
#include "Encoder2.h"
#include "Adafruit_SI5351.h"
#include "Wire.h"
#include "rox2x.h"
#include "eeprom_2m.h"

static const int BUTTON1_Pin = 10;
static const int BUTTON2_Pin = 11;
static const int BUTTON3_Pin = 12;
static const int BUTTON4_Pin = 13;
static const int ENCODER_I_Pin = 14;
static const int ENCODER_Q_Pin = 15;
static const int LCD_RS_Pin = 16;
static const int LCD_EN_Pin = 17;
static const int LCD_D4_Pin = 18;
static const int LCD_D5_Pin = 19;
static const int LCD_D6_Pin = 20;
static const int LCD_D7_Pin = 21;
static const int RSSI_Pin = A0;
static const int DIV_9V_Pin = A1;

static const int LCD_WIDTH = 16;
static const int LCD_HEIGHT = 2;

static const int MENU_DLY_MS = 1000;  // Number of millisseconds to hold button to enter menu
static const int EDIT_DLY_MS = 10000; // Number of millisseconds until frequency edit is locked after last action

static const int MIN_FREQ_EDIT = 1000; // Hz, lowest increment when editing frequencies (the passband is about 50 kHz wide)
static const int MAX_FREQ_EDIT = 100000; // Hz, highest increment when editing frequencies
static const int MAX_TUNING_ERROR = 0; // Hz, maxmimum allowed deviation from ideal tuning frequency.
// There was speculation that higher values might give lower jitter since that allows a lower
// denominator, but actual measurements of spectra does not give a clear indication that that
// would be better.



// Voltage divider of 240k and 91k, 3.3 V ADC reference, 10 bits resolution and a fudge factor
static const float BATT_SCALE = 1.03*3.3*(240 + 91)/91/1024; 
static const float BATT_HIGH = 8.8; // Voltage regarded as full battery
static const float BATT_LOW = 6.5;  // Voltage regarded as empty battery
static const float BATT_LEVELS = 7; // Number of levels of battery indicator, one is below low and one is above high
static const float BATT_STEP = (BATT_HIGH - BATT_LOW)/(BATT_LEVELS - 2);


LiquidCrystal lcd(LCD_RS_Pin, LCD_EN_Pin, LCD_D4_Pin, LCD_D5_Pin, LCD_D6_Pin, LCD_D7_Pin);

Adafruit_SI5351 si5351 = Adafruit_SI5351();

Bounce button1 = Bounce();
Bounce button2 = Bounce();
Bounce button3 = Bounce();
Bounce button4 = Bounce();

uint32_t freq;  // Hz
uint32_t frequencies[10]; // Hz, read from EEPROM at power up, can be modified during use. 1-based, so only index 1-9 are used.
uint32_t max_frequency_idx = 1; // Highest used pre-programmed (EEPROM) frequency index
uint32_t current_freq_idx = 1;

static const int PLL_DELAY = 30; // Number of ms to wait after retuning a PLL

// Frequency sweeping parameters
static const uint SWEEP_FREQ_MIN = 144000000; // Hz
static const uint SWEEP_FREQ_MAX = 146000000;
static const uint SWEEP_FREQ_STEP_COARSE = 10000;
static const uint SWEEP_FREQ_STEP_FINE = 2000;
static const uint SWEEP_RSSI_AVG = 1000; // Number of RSSI measurements to average during sweeping


static const uint32_t IF_FREQ = 10700000; // Hz
int i2c_found;

void isr_a(void);
void isr_b(void);
void tune_pll(uint32_t pll_freq, uint32_t tolerance, si5351PLL_t pll);
void tune_freq(uint32_t freq);
void step_frequency();
float read_rssi(int n = 1);
int update_rssi();
float update_batt(uint32_t period_ms = 200);
void lcd_print_freq(uint32_t freq);
char *format_MHz(uint64_t freq);
void enable_test_source(uint32_t freq);
void disable_test_source();
void show_sweep_freq(uint32_t sweep_freq);


Encoder2 rot_enc(ENCODER_I_Pin, ENCODER_Q_Pin, isr_a, isr_b);

// Special LCD character codes
static const byte CHAR_BLOCK = 0; // Full block, used for RSSI
static const byte CHAR_RSSI = 1;  // Modifed block, used for top of RSSI
static const byte CHAR_BATT = 2;  // Modified battery level symbol
static const byte CHAR_RIGHT_ARROW = 3;  // Arrow pointing right
static const byte CHAR_ENTER = 4; // "Enter" sign

// Modifiable glyph for RSSI indication, starts out as a full block
byte glyph[8] = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

// Glyphs for battery indication
byte batt_glyphs[7][8] = {
  {0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11111},
  {0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11111, 0b11111},
  {0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b11111, 0b11111, 0b11111},
  {0b01110, 0b10001, 0b10001, 0b10001, 0b11111, 0b11111, 0b11111, 0b11111},
  {0b01110, 0b10001, 0b10001, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111},
  {0b01110, 0b10001, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111},
  {0b01110, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111}
};

// Right arrow for use in menu
byte right_arrow_glyph[8] = {
  0b00000,
  0b00100,
  0b00010,
  0b11111,
  0b00010,
  0b00100,
  0b00000,
  0b00000
};

// "Enter" arrow for use in menu
byte enter_glyph[8] = {
  0b00000,
  0b00001,
  0b00001,
  0b01001,
  0b11111,
  0b01000,
  0b00000
};


void setup() {
  Serial.begin(115200);

  rp2040.enableDoubleResetBootloader();
  eeprom_init();
  max_frequency_idx = recall_frequencies(frequencies);
  current_freq_idx = 1;
  freq = frequencies[current_freq_idx];

  rot_enc.begin();
  button1.attach(BUTTON1_Pin, INPUT_PULLUP);
  button1.interval(10);
  button2.attach(BUTTON2_Pin, INPUT_PULLUP);
  button2.interval(10);
  button3.attach(BUTTON3_Pin, INPUT_PULLUP);
  button3.interval(10);
  button4.attach(BUTTON4_Pin, INPUT_PULLUP);
  button4.interval(10);

  si5351.begin();
  si5351.powerDownOutput(0, false);
  si5351.enableOutput(0, true);
  si5351.outputDrive(0, SI5351_DRIVE_2MA);
  si5351.outputDrive(1, SI5351_DRIVE_2MA);
  si5351.outputDrive(2, SI5351_DRIVE_2MA);
  si5351.invertOutput(2, true);
  si5351.enableOutput(1, false);
  si5351.enableOutput(2, false);
  si5351.setupMultisynthInt(0, SI5351_PLL_A, SI5351_MULTISYNTH_DIV_6);
  si5351.setupMultisynthInt(1, SI5351_PLL_B, SI5351_MULTISYNTH_DIV_6);
  si5351.setupMultisynthInt(2, SI5351_PLL_B, SI5351_MULTISYNTH_DIV_6);
  si5351.powerDownOutput(1, true);
  si5351.powerDownOutput(2, true);

  delay(PLL_DELAY);
  tune_freq(freq);
  si5351.resetPLL(SI5351_PLL_A);
  tune_pll(6*freq, 6*MAX_TUNING_ERROR, SI5351_PLL_B);
  si5351.resetPLL(SI5351_PLL_B);
  delay(PLL_DELAY);

  // set up the LCD's number of columns and rows:
  lcd.begin(LCD_WIDTH, LCD_HEIGHT);
  // Define custom characters
  lcd.createChar(CHAR_BLOCK, glyph);
  lcd.createChar(CHAR_BATT, batt_glyphs[6]);
  lcd.createChar(CHAR_RIGHT_ARROW, right_arrow_glyph);
  lcd.createChar(CHAR_ENTER, enter_glyph);

  // The following does not seem to work. Maybe better low-power functionality will be
  // added in the future. For now, the best thing to do seems to be to set the 
  // clock frequency to 50 MHz in the IDE.

  // Set clock frequency to 15.43 MHz to save power
  // set_sys_clock_khz(15429, false); 
  // Reduce core voltage
  // vreg_set_voltage(VREG_VOLTAGE_1_00); // 0.85V did not work, 0.95V seems pretty stable
}



// States of the "UI" state machine
typedef enum {
  S_FREQ_INIT, // Setup LO PLL
  S_FREQ_EDIT, // Rotating the dial adjusts the frequency
  S_BUTTON_LOCK, // Rotating the dial does not affect the frequency, button 1 needs to be pressed to unlock
  S_FREQ_MENU_DLY, // Waiting for button 4 to be pressed long enough to enter the menu
  S_MENU_FREQ1, // Init first menu (frequency edit)
  S_MENU_FREQ2, // Edit frequencies
  S_MENU_SHOW_SAVED, // Show "saved" message
  S_MENU_SHOW_CANCEL, // Show "cancelled" message
  S_MENU_BATT1, // Show battery voltage 1
  S_MENU_BATT2, // Show battery voltage 2
  S_MENU_SWEEP_RESP0, // Sweep passband reponse?
  S_MENU_SWEEP_RESP1, // Sweep passband!
  S_MENU_SWEEP_RESP2, // Sweep  
  S_MENU_SWEEP_RESP3, // Sweep  
  S_MENU_SWEEP_RESP4, // Sweep  
  S_MENU_SWEEP_RESP5, // Sweep  
  S_MENU_SWEEP_RESP6, // Sweep  
  S_MENU_SWEEP_RESP7, // Sweep  
  S_MENU_SWEEP_RESP_DONE1, // After sweep passband 
  S_MENU_SWEEP_RESP_DONE2, // After sweep  
  S_MENU_SWEEP_RESP_DONE3, // After sweep  
  S_MENU_SWEEP_BAND0, // Sweep entire band to e.g. listen for internal noise?
  S_MENU_SWEEP_BAND1, // Sweep
  S_MENU_SWEEP_BAND2, // Sweep
  S_MENU_SWEEP_BAND3, // Sweep
  S_MENU_SWEEP_BAND_DONE1, // After band sweep  
  S_MENU_SWEEP_BAND_DONE2 // After band sweep  
} uiState_t;


void loop()
{
  static uint64_t f_increment = 10000;  // Current rotary encoder frequency increment
  static int force_freq_update = 1;
  static uiState_t state = S_FREQ_INIT;
  static uint32_t last_button_time;
  static int edit_freq_idx;
  static uint32_t sweep_freq;
  int rot_temp;
  int cursor_has_moved = 0;  
  
  if(state < S_MENU_FREQ1) {
    // Only update RSSI etc if we are not in the menu
    cursor_has_moved = update_rssi();
    update_batt();
  }

  // GUI state machine
  if(state == S_FREQ_INIT) { // ##################################    S_FREQ_INIT
    si5351.powerDownOutput(0, false);
    si5351.outputDrive(0, SI5351_DRIVE_2MA);
    si5351.enableOutput(0, true);
    si5351.setupMultisynthInt(0, SI5351_PLL_A, SI5351_MULTISYNTH_DIV_6);
    tune_freq(freq);
    si5351.resetPLL(SI5351_PLL_A);
    delay(PLL_DELAY);
    state = S_FREQ_EDIT;

  } else if(state == S_FREQ_EDIT) { // ##################################    S_FREQ_EDIT
    button1.update();
    if(button1.fell()) {
      f_increment *= 10;
      last_button_time = millis();
      cursor_has_moved = 1;
    }
    button2.update();
    if(button2.fell()) {
      f_increment /= 10;
      last_button_time = millis();
      cursor_has_moved = 1;
    }
    if(f_increment > MAX_FREQ_EDIT) {
      f_increment = MIN_FREQ_EDIT;
    }
    if(f_increment < MIN_FREQ_EDIT) {
      f_increment = MAX_FREQ_EDIT;
    }
    button3.update();
    if(button3.fell()) {
      // Step to the next stored frequency.
      last_button_time = millis();
      step_frequency();
      force_freq_update = 1;  
    }
    button4.update();
    if(button4.fell()) {
      // Button 4 was pressed, we might be entering the menu
      last_button_time = millis();
      lcd.setCursor(14, 1);
      lcd.print(" M");
      state = S_FREQ_MENU_DLY;
    }
    rot_temp = rot_enc.read();
    if (rot_temp != 0 || force_freq_update) {
      force_freq_update = 0;
      last_button_time = millis();
      freq += rot_temp*f_increment;
      frequencies[current_freq_idx] = freq;
      rot_enc.write(0);
      tune_freq(freq);
      lcd.setCursor(0, 1);
      lcd_print_freq(freq);
      lcd.setCursor(15, 1);
      lcd.print(current_freq_idx); // Index of current frequency
      cursor_has_moved = 1;
      Serial.print(format_MHz(freq));
      Serial.print(" #");
      Serial.println(current_freq_idx);
    }
    if(cursor_has_moved) {
      // Enable and position the cursor to mark which digit of the frequency we can adjust.
      if(f_increment >= 100000) {
        lcd.setCursor(4, 1);
      } else if(f_increment >= 10000) {
        lcd.setCursor(5, 1);
      } else if(f_increment >= 1000) {
        lcd.setCursor(6, 1);
      } else {
        lcd.setCursor(8, 1);
      }
      lcd.cursor();
    }

    if(millis() > last_button_time + EDIT_DLY_MS) {
      // Button time out, lock the frequency
      state = S_BUTTON_LOCK;
      lcd.noCursor();
    }
 
  } else if(state == S_BUTTON_LOCK) { // ##################################    S_BUTTON_LOCK
    // The encoder is ignored in this state, need to press button 1 to unlock
    rot_enc.write(0); // Do not collect encoder clicks in this state
    button1.update();
    if(button1.fell()) {
      last_button_time = millis();
      lcd.clear();
      force_freq_update = 1;
      state = S_FREQ_EDIT;
    }

  } else if(state == S_FREQ_MENU_DLY) { // ##################################    S_FREQ_MENU_DLY
    // Button 4 has been pressed, we are waiting to see 
    // if it is held down long enough to enter the menu.
    button4.update();
    if(button4.read() == HIGH) {
      // The button was released early, go back to edit state
      last_button_time = millis();
      lcd.setCursor(14, 1);
      lcd.print("  "); // Overwrite menu countdown.
      state = S_FREQ_EDIT;
    } else {
      // Still holding the button
      if(millis() > last_button_time + MENU_DLY_MS) {
        // The button has been held long enough to enter the menu.
        f_increment = 100000;
        state = S_MENU_FREQ1;
      }
    }

  } else if(state == S_MENU_FREQ1) { // ##################################    S_MENU_FREQ1
    // Show the first menu page
    lcd.clear();
    lcd.setCursor(0, 1);
    // Soft buttons:
    // 1: Right (change digit)
    // 2: Next frequency
    // 3: Store this and previous frequencies (but not those with higher indices)
    // 4: Cancel the frequency edit
    lcd.print("1:- 2:+ 3:v  4:X"); 
    lcd.setCursor(2, 1);
    lcd.write(CHAR_RIGHT_ARROW); // Replace '-' with right arrow
    lcd.setCursor(10, 1);
    lcd.write(CHAR_ENTER); // Replace 'v' with enter arrow
    edit_freq_idx = 1;
    force_freq_update = 1;
    state = S_MENU_FREQ2;

  } else if(state == S_MENU_FREQ2) { // ##################################    S_MENU_FREQ2
    // Edit stored frequencies
    button1.update();
    if(button1.fell()) {
      // Change which digit we are editing
      f_increment /= 10;
      if(f_increment < MIN_FREQ_EDIT) {
        f_increment = MAX_FREQ_EDIT;
      }
      cursor_has_moved = 1;
    }
    button2.update();
    if(button2.fell()) {
      // Go to next frequency
      edit_freq_idx++;
      force_freq_update = 1;
      if(edit_freq_idx > 9) {
        edit_freq_idx = 1;
      }
      if(frequencies[edit_freq_idx] < MIN_TUNE_FREQ) {
        frequencies[edit_freq_idx] = DEFAULT_TUNE_FREQ;
      }
      if(frequencies[edit_freq_idx] > MAX_TUNE_FREQ) {
        frequencies[edit_freq_idx] = DEFAULT_TUNE_FREQ;
      }      
    }
    button3.update();
    if(button3.fell()) {
      // Store the frequencies up to and including the current one
      if(edit_freq_idx < 9) {
        frequencies[edit_freq_idx+1] = 0; // Make next frequency invalid to signal the number of valid frequencies
      }
      store_frequencies(frequencies);
      max_frequency_idx = edit_freq_idx;
      freq = frequencies[1];
      current_freq_idx = 1;
      state = S_MENU_SHOW_SAVED;
    }
    button4.update();
    if(button4.fell()) {
      // Cancel the storing of frequencies
      state = S_MENU_SHOW_CANCEL;
    }
    rot_temp = rot_enc.read();
    if (rot_temp != 0 || force_freq_update) {
      force_freq_update = 0;
      frequencies[edit_freq_idx] += rot_temp*f_increment;
      if(frequencies[edit_freq_idx] < MIN_TUNE_FREQ) {
        frequencies[edit_freq_idx] = MIN_TUNE_FREQ;
      }
      if(frequencies[edit_freq_idx] > MAX_TUNE_FREQ) {
        frequencies[edit_freq_idx] = MAX_TUNE_FREQ;
      }
      rot_enc.write(0);
      lcd.setCursor(0, 0);
      lcd_print_freq(frequencies[edit_freq_idx]);
      lcd.setCursor(15, 0);
      lcd.print(edit_freq_idx); // Index of current frequency
      cursor_has_moved = 1;
      Serial.print(format_MHz(frequencies[edit_freq_idx]));
      Serial.print(" #");
      Serial.println(edit_freq_idx);
    }
    if(cursor_has_moved) {
      // Enable and position the cursor to mark which digit of the frequency we can adjust.
      if(f_increment >= 100000) {
        lcd.setCursor(4, 0);
      } else if(f_increment >= 10000) {
        lcd.setCursor(5, 0);
      } else if(f_increment >= 1000) {
        lcd.setCursor(6, 0);
      } else {
        lcd.setCursor(8, 0);
      }
      lcd.cursor();
    }

  } else if(state == S_MENU_SHOW_SAVED) { // ##################################    S_MENU_SHOW_SAVED
    // Show saved message
    lcd.clear();
    lcd.noCursor();
    lcd.setCursor(0, 0);
    lcd.print(edit_freq_idx);
    lcd.print(" frequencies");
    lcd.setCursor(0, 1);
    lcd.print("saved");
    delay(3000);
    state = S_MENU_BATT1;

  } else if(state == S_MENU_SHOW_CANCEL) { // ##################################    S_MENU_SHOW_CANCEL
    // Show cancelled message
    lcd.clear();
    lcd.noCursor();
    lcd.print("Save cancelled");
    delay(500);
    state = S_MENU_BATT1;

  } else if(state == S_MENU_BATT1) { // ##################################    S_MENU_BATT1
    // Show battery voltage, step 1
    float batt;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Battery: ");
    batt = BATT_SCALE * analogRead(DIV_9V_Pin);
    lcd.print(batt);
    lcd.print(" V ");
    lcd.setCursor(0, 1);
    lcd.write(byte(CHAR_BATT));
    lcd.print("         4:Next");
    state = S_MENU_BATT2;

  } else if(state == S_MENU_BATT2) {
    // Show battery voltage, step 2
    float batt;
    batt = update_batt(200);
    if(batt > 0) {
      lcd.setCursor(9, 0);
      lcd.print(batt);
    }
    button4.update();
    if(button4.fell()) {
      // Next
      force_freq_update = 1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sweep response?");
      lcd.setCursor(0, 1);
      lcd.print("1:Sweep   4:Skip");
      enable_test_source(freq); // Turn on the test frequency to facilitate volume adjustment
      state = S_MENU_SWEEP_RESP0;
    }

  } else if(state == S_MENU_SWEEP_RESP0) { // ##################################    S_MENU_SWEEP_RESP0
    // Do a frequency sweep?
    button1.update();
    if(button1.fell()) {
      state = S_MENU_SWEEP_RESP1;
    }
    button4.update();
    if(button4.fell()) {
      disable_test_source();
      state = S_MENU_SWEEP_BAND0;
    }

  } else if(state == S_MENU_SWEEP_RESP1) { // ##################################    S_MENU_SWEEP_RESP1
    // Do a frequency sweep
    sweep_freq = freq - 2*IF_FREQ - 200000; // Check image rejection
    enable_test_source(sweep_freq);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sweeping...");
    Serial.println("Sweeping passband shape");
    state = S_MENU_SWEEP_RESP2;

  } else if(state == S_MENU_SWEEP_RESP2) { // ##################################    S_MENU_SWEEP_RESP2
    // Doing a frequency sweep. Checking image rejection below image passband.
    tune_pll(6 * sweep_freq, 6*MAX_TUNING_ERROR, SI5351_PLL_B); // Tune, but do not reset the PLL
    show_sweep_freq(sweep_freq);
    Serial.print(sweep_freq);
    Serial.print(";");
    delay(PLL_DELAY);
    Serial.println(rssi_to_dbm(read_rssi(SWEEP_RSSI_AVG)));
    sweep_freq += SWEEP_FREQ_STEP_COARSE;
    if(sweep_freq > freq - 2*IF_FREQ - 40000) {
      sweep_freq -= SWEEP_FREQ_STEP_COARSE - SWEEP_FREQ_STEP_FINE;
      state = S_MENU_SWEEP_RESP3;
    }

  } else if(state == S_MENU_SWEEP_RESP3) { // ##################################    S_MENU_SWEEP_RESP3
    // Doing a frequency sweep. Checking image rejection in image passband.
    tune_pll(6 * sweep_freq, 6*MAX_TUNING_ERROR, SI5351_PLL_B); // Tune, but do not reset the PLL
    show_sweep_freq(sweep_freq);
    Serial.print(sweep_freq);
    Serial.print(";");
    delay(PLL_DELAY);
    Serial.println(rssi_to_dbm(read_rssi(SWEEP_RSSI_AVG)));
    sweep_freq += SWEEP_FREQ_STEP_FINE;
    if(sweep_freq > freq - 2*IF_FREQ + 40000) {
      sweep_freq -= SWEEP_FREQ_STEP_FINE - SWEEP_FREQ_STEP_COARSE;
      state = S_MENU_SWEEP_RESP4;
    }

} else if(state == S_MENU_SWEEP_RESP4) { // ##################################    S_MENU_SWEEP_RESP4
    // Doing a frequency sweep. Checking image rejection above image passband.
    tune_pll(6 * sweep_freq, 6*MAX_TUNING_ERROR, SI5351_PLL_B); // Tune, but do not reset the PLL
    show_sweep_freq(sweep_freq);
    Serial.print(sweep_freq);
    Serial.print(";");
    delay(PLL_DELAY);
    Serial.println(rssi_to_dbm(read_rssi(SWEEP_RSSI_AVG)));
    sweep_freq += SWEEP_FREQ_STEP_COARSE;
    if(sweep_freq > freq - 2*IF_FREQ + 200000) {
      sweep_freq = freq - 200000;
      Serial.println(); // Mark jump from image to real band with an empty line.
      state = S_MENU_SWEEP_RESP5;
    }

} else if(state == S_MENU_SWEEP_RESP5) { // ##################################    S_MENU_SWEEP_RESP5
    // Doing a frequency sweep. Checking rejection below passband.
    tune_pll(6 * sweep_freq, 6*MAX_TUNING_ERROR, SI5351_PLL_B); // Tune, but do not reset the PLL
    show_sweep_freq(sweep_freq);
    Serial.print(sweep_freq);
    Serial.print(";");
    delay(PLL_DELAY);
    Serial.println(rssi_to_dbm(read_rssi(SWEEP_RSSI_AVG)));
    sweep_freq += SWEEP_FREQ_STEP_COARSE;
    if(sweep_freq > freq - 40000) {
      sweep_freq -=  SWEEP_FREQ_STEP_COARSE - SWEEP_FREQ_STEP_FINE;
      state = S_MENU_SWEEP_RESP6;
    }

  } else if(state == S_MENU_SWEEP_RESP6) { // ##################################    S_MENU_SWEEP_RESP6
    // Doing a frequency sweep. Checking passband.
    tune_pll(6 * sweep_freq, 6*MAX_TUNING_ERROR, SI5351_PLL_B); // Tune, but do not reset the PLL
    show_sweep_freq(sweep_freq);
    Serial.print(sweep_freq);
    Serial.print(";");
    delay(PLL_DELAY);
    Serial.println(rssi_to_dbm(read_rssi(SWEEP_RSSI_AVG)));
    sweep_freq += SWEEP_FREQ_STEP_FINE;
    if(sweep_freq > freq + 40000) {
      sweep_freq -= SWEEP_FREQ_STEP_FINE - SWEEP_FREQ_STEP_COARSE;
      state = S_MENU_SWEEP_RESP7;
    }

} else if(state == S_MENU_SWEEP_RESP7) { // ##################################    S_MENU_SWEEP_RESP7
    // Doing a frequency sweep. Checking rejection above passband.
    tune_pll(6 * sweep_freq, 6*MAX_TUNING_ERROR, SI5351_PLL_B); // Tune, but do not reset the PLL
    show_sweep_freq(sweep_freq);
    Serial.print(sweep_freq);
    Serial.print(";");
    delay(PLL_DELAY);
    Serial.println(rssi_to_dbm(read_rssi(SWEEP_RSSI_AVG)));
    sweep_freq += SWEEP_FREQ_STEP_COARSE;
    if(sweep_freq > freq + 200000) {
      sweep_freq = freq;
      state = S_MENU_SWEEP_RESP_DONE1;
    }    

  } else if(state == S_MENU_SWEEP_RESP_DONE1) { // ##################################    S_MENU_SWEEP_RESP_DONE1
    // turn off CLK1 and CLK2
    disable_test_source();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Done!");
    lcd.setCursor(0, 1);
    lcd.print("1:Rep 2:Fon  4:X");
    state = S_MENU_SWEEP_RESP_DONE2;

  } else if(state == S_MENU_SWEEP_RESP_DONE2) { // ##################################    S_MENU_SWEEP_RESP_DONE2
    button1.update();
    if(button1.fell()) {
      // Repeat sweep
      state = S_MENU_SWEEP_RESP1;
    }
    button2.update();
    if(button2.fell()) {
      // Toggle test source
      enable_test_source(sweep_freq);
      lcd.setCursor(0, 1);
      lcd.print("1:Rep 2:Foff 4:X");
      state = S_MENU_SWEEP_RESP_DONE3;
    }
    button4.update();
    if(button4.fell()) {
      // Exit
      disable_test_source();
      state = S_MENU_SWEEP_BAND0;
    }

  } else if(state == S_MENU_SWEEP_RESP_DONE3) { // ##################################    S_MENU_SWEEP_RESP_DONE3
    button1.update();
    if(button1.fell()) {
      // Repeat sweep
      state = S_MENU_SWEEP_RESP1;
    }
    button2.update();
    if(button2.fell()) {
      // Toggle test source
      disable_test_source();
      lcd.setCursor(0, 1);
      lcd.print("1:Rep 2:Fon  4:X");
      state = S_MENU_SWEEP_RESP_DONE2;
    }
    button4.update();
    if(button4.fell()) {
      // Exit
      disable_test_source();
      state = S_MENU_SWEEP_BAND0;
    }

  } else if(state == S_MENU_SWEEP_BAND0) { // ##################################    S_MENU_SWEEP_BAND0
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sweep band?");
    lcd.setCursor(0, 1);
    lcd.print("1:Sweep   4:Skip");
    state = S_MENU_SWEEP_BAND1;

  } else if(state == S_MENU_SWEEP_BAND1) { // ##################################    S_MENU_SWEEP_BAND1
    // Do a frequency sweep over the entire band?
    button1.update();
    if(button1.fell()) {
      state = S_MENU_SWEEP_BAND2;
    }
    button4.update();
    if(button4.fell()) {
      // Skip
      lcd.clear();
      state = S_FREQ_INIT;
    }

  } else if(state == S_MENU_SWEEP_BAND2) { // ##################################    S_MENU_SWEEP_BAND2
    // Do a frequency sweep over the band
    sweep_freq = MIN_TUNE_FREQ;
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Sweeping...  4:X");
    Serial.println("Sweeping band for noise");
    state = S_MENU_SWEEP_BAND3;

  } else if(state == S_MENU_SWEEP_BAND3) { // ##################################    S_MENU_SWEEP_BAND3
    // Doing a frequency sweep. Checking noise in band.
    tune_freq(sweep_freq); // Tune, but do not reset the PLL
    show_sweep_freq(sweep_freq);
    Serial.print(sweep_freq);
    Serial.print(";");
    delay(PLL_DELAY);
    Serial.println(rssi_to_dbm(read_rssi(SWEEP_RSSI_AVG)), 5);
    sweep_freq += SWEEP_FREQ_STEP_COARSE;
    if(sweep_freq > MAX_TUNE_FREQ) {
      state = S_MENU_SWEEP_BAND_DONE1;
    }
    button4.update();
    if(button4.fell()) {
      // Cancel
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Canceled");
      delay(500);
      tune_freq(freq); // Restore original frequency
      lcd.clear();
      state = S_FREQ_INIT;
    }

  } else if(state == S_MENU_SWEEP_BAND_DONE1) { // ##################################    S_MENU_SWEEP_BAND_DONE1
    tune_freq(freq); // Restore original frequency
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Done!");
    lcd.setCursor(0, 1);
    lcd.print("1:Repeat     4:X");
    state = S_MENU_SWEEP_BAND_DONE2;

  } else if(state == S_MENU_SWEEP_BAND_DONE2) { // ##################################    S_MENU_SWEEP_BAND_DONE2
    button1.update();
    if(button1.fell()) {
      // Repeat sweep
      state = S_MENU_SWEEP_BAND1;
    }
    button4.update();
    if(button4.fell()) {
      // Exit
      lcd.clear();
      state = S_FREQ_INIT;
    }

  } else {
    // This should never happen
    force_freq_update = 1;
    state = S_FREQ_INIT;
  }
  
}


void show_sweep_freq(uint32_t sweep_freq)
{    
    lcd.setCursor(0, 1);
    lcd.print(format_MHz(sweep_freq));
}


// Read the RSSI signal a number of times and return the average
float read_rssi(int n)
{
  float sum = 0;

  for(int ii = 0; ii < n; ii++) {
    sum += analogRead(RSSI_Pin);
  }
  return sum/n;
}


// Approximate receive level in dBm for a given RSSI
// RSSI gain is 1+22/39 = 1.564
// ADC has 1024 codes at 3.3 V, i.e. 310.3 codes/V
// -90 dBm nominally gives 0.6 V * 1.564 = 0.938 V (see datasheet plot)
// -40 dBm nominally gives 1.4 V * 1.564 = 2.19 V (see datasheet plot)
// Hence: dBm = 0.129 * RSSI - 127.5
// But the bias at the input is adjustable, so the absolute level is lost and meaningless.
// Use a term that makes the output go from about 0 to positive values.
float rssi_to_dbm(float rssi)
{
  return 0.12877 * (float) rssi - 10;
}


// Update the rssi status indicator.
// Return true if the display was updated.
int update_rssi()
{
  static uint32_t n_avg;
  static uint32_t rssi_sum;
  static uint32_t last_rssi_time;
  byte mask;
  int col;
  int rssi;
  int i;
  int debug = 0;
  
  rssi_sum += analogRead(RSSI_Pin);
  n_avg++;
  if(millis() - last_rssi_time > 200) {
    // About 200 ms has passed since last RSSI update on the display. Update it again.
    col = 0;    
    lcd.noCursor();
    rssi = rssi_sum/n_avg;
    lcd.setCursor(0, 0);
    if(debug) {
      Serial.println(rssi);
    }
    if(rssi > 40*16) {
      // Maxing out the display, probably should not happen
      rssi = 40*16;
    }
    // Reserve the first character for battery indication
    rssi -= 40;    
    lcd.write(byte(CHAR_BATT));  // Battery indicator
    col++;

    while(rssi >= 40) {
      lcd.write(byte(CHAR_BLOCK));  // Filled block
      if(debug) {
        Serial.print("#");
      }
      col++;
      rssi -= 40;
    }
    if(debug) {
      Serial.print("RSSI = ");
      Serial.print(rssi);
      Serial.print(" n_avg  = ");
      Serial.println(n_avg);
    }
    if(rssi > 0) {
      // Create a special character with between 0 and 39 filled pixels 
      // to make up for the rest of the RSSI.
      for(i = 0; i <= 7; i++) {
        // Clear glyph
        glyph[i] = 0;
      }
      mask = 0b10000;
      i = 7;
      while(rssi >= 8) {
        for(i = 0; i <= 7; i++) {
          // Fill columns from the left
          glyph[i] |= mask;
        }
        mask = mask>>1;
        rssi -= 8;
      }
      for(i = 0; i < rssi; i++) {
        // Put remaining dots in leftmost empty column, starting from the bottom
        glyph[7-i] |= mask;
      }
      lcd.createChar(CHAR_RSSI, glyph);
      lcd.setCursor(col, 0); // createChar() messes up the cursor position, need to set it again
      lcd.write(byte(CHAR_RSSI));
      col++;
      while(col++ < 16) {
        // Clear the rest of the line
        lcd.write(' ');
      }
    }

    rssi_sum = 0;
    n_avg = 0;
    last_rssi_time = millis();
    return 1;
  }
  return 0;
}

// Update the battery status indicator glyph.
// Optional parameter period_ms is the minimum period between updates. Set to zero if an immediate
// update is desired. 
// Call this repeatedly. An average is calculated over the readings from the calls until it is time to update.
// Return the voltage if an update was made. Otherwise a negative number.
float update_batt(uint32_t period_ms)
{
  static uint32_t n_avg;
  static uint32_t batt_sum;
  static uint32_t last_batt_time;
  static int batt_bin, last_batt_bin = 10;        // Battery level bin
  static int batt_print_count = 0;
  float batt;
  int debug = 0;

  batt_sum += analogRead(DIV_9V_Pin);
  n_avg++;
  if(millis() - last_batt_time > period_ms) {
    batt = BATT_SCALE * batt_sum/n_avg;
    batt_bin = floor((batt - BATT_LOW)/(float)(BATT_STEP)) + 1.0;
    if(batt_bin < 0) {
      batt_bin = 0;
    } else if(batt_bin > BATT_LEVELS - 1) {
      batt_bin = BATT_LEVELS - 1;
    }

    if(batt_bin != last_batt_bin) {
      // Update the battery indicator      
      lcd.createChar(CHAR_BATT, batt_glyphs[batt_bin]);
      // Assume the character was already printed, so no need to print it again, 
      // just change its appearance.
    }
    last_batt_bin = batt_bin;
    if(batt_print_count++ > 10 && debug) {
      Serial.print("Battery: ");
      Serial.print(batt);
      Serial.print(" V");
      Serial.print("BATT_SCALE*1000: ");
      Serial.println(BATT_SCALE*1000);
      batt_print_count = 0;
    }
    batt_sum = 0;
    n_avg = 0;
    last_batt_time = millis();
    return batt;
  }
  return -1.0;
}


// Step to the next stored frequency. If the current frequency has been modified from the EEPROM value,
// it is remembered (in RAM) as the frequency for the current spot. 
void step_frequency()
{
  frequencies[current_freq_idx] = freq;
  current_freq_idx++;
  if(current_freq_idx > max_frequency_idx) {
    current_freq_idx = 1;
  }
  freq = frequencies[current_freq_idx];
}


char *format_MHz(uint64_t freq)
{
  static char str[50];
  uint64_t f;
  int Hz, kHz, MHz;

  f = freq;
  Hz = f % 1000;
  f -= Hz;
  f /= 1000;
  kHz = f % 1000;
  f -= kHz;
  f /= 1000;
  MHz = f;

  sprintf(str, "%03d.%03d %03d", MHz, kHz, Hz);
  return str;  
}


// Print a frequency on the LCD, assuming it is between 100 MHz and 999 MHz
void lcd_print_freq(uint32_t freq)
{
  lcd.print(format_MHz(freq));
}


// Tune a PLL of the Si5351 to a specific frequency (Hz) within some tolerance (Hz).
// Use the smallest numerator possible to get within the desired tolerance.
// Integer PLL mode will be used when possible.
// With a 25 MHz reference, the worst-case resolution of the PLL is about 
// 1.5 Hz (25e5/2^20), but for most frequencies much better.
// So a maximum nominal error of 1 Hz can be guaranteed. There was an idea that 
// allowing higher tolerance on the frequency would result in less jitter as 
// smaller numerators can be used, but this is not evident from preliminary 
// measurements of the resulting spectra, so setting tolerance to 0 to get the best
// possible fit does probably not come with much of a downside.
void tune_pll(uint32_t pll_freq, uint32_t tolerance, si5351PLL_t pll)
{
  uint32_t int_mult; // Integer part of feedback multiplier
  uint32_t rem_mult; // Remainder of feedback multiplier
  double err, target, mediant, err_target; // float does not have enough resolution 
                                           // to deal with single-digit differences 
                                           // between numbers above 10^8.
  uint32_t a = 0, b = 1, c = 1, d = 1, ac = 0, bd = 1;
  int debug = 0;

  int_mult = pll_freq/SI5351_CRYSTAL_FREQ_25MHZ; // Integer part of feedback multiplier
  rem_mult = pll_freq - int_mult*SI5351_CRYSTAL_FREQ_25MHZ;

  // Calculate the requirement on the error of the approximation of the
  // feedback divider to get less than the specified tolerance.
  err_target = tolerance/(double)SI5351_CRYSTAL_FREQ_25MHZ; 

  target = rem_mult/(double)SI5351_CRYSTAL_FREQ_25MHZ;
  mediant = 0;
  if(target < err_target) {
    // target is 0, or very close to it, try to use integer PLL mode
    si5351.setupPLLInt(pll, int_mult, false);
  } else if(1-target < err_target) {
    // target is very close to 1, try to use integer PLL mode
    int_mult++;
    si5351.setupPLLInt(pll, int_mult, false);
  } else {
    // The target frequency is not close enough to a value that allows us to use integer PLL mode.
    // Use Farey fractions to find the rational approximation with the smallest denominator 
    // that approximates the target to within the required tolerance.
    // See https://web.archive.org/web/20181119092100/https://nrich.maths.org/6596
    // a, b, c, d notation from https://en.wikipedia.org/wiki/Farey_sequence is used here
    // (not from the above reference).
    // I.e. narrow the interval between a/b and c/d until we are close enough with either endpoint,
    // or we have a numerator that is bigger than what is allowed.
    // Start with the interval 0/1 to 1/1 (i.e. 0 to 1). We know from the previous checks that neither of 
    // the end points are good enough, so we do not need to check for that case again.

    err = err_target + 1; // err just needs to be bigger than err_target when we enter the loop
    int ii = 0;
    while(err > err_target) {
      ac = a+c;
      bd = b+d;
      if(bd >= 1<<20) {
        // Cannot have numerators this large in Si5351.
        // Use the best approximation of the previous iteration.
        if(target - a/(double)b < c/(double)d - target) {
          ac = a;
          bd = b;
        } else {
          ac = c;
          bd = d;
        }
        break;
      }
      mediant = ac/(double)bd;
      if(target < mediant) {
        c = ac;
        d = bd;
        // a/b might be a better approximation, but it was not good enough, 
        // so we need to put our hope in the mediant.
        err = mediant - target;
      } else {
        a = ac;
        b = bd;
        // c/d might be a better approximation, but it was not good enough, 
        // so we need to put our hope in the mediant.
        err = target - mediant;
      }
      if(ii++ > 200) {
        // Emergency exit, the loop does not seem to terminate by itself. This should not happen.
        break;
      }
    }
    si5351.setupPLL(pll, int_mult, ac, bd, false); // Do not reset the PLL!
  }
  double actual_freq;

  if(debug) {
    actual_freq = SI5351_CRYSTAL_FREQ_25MHZ * (int_mult + mediant);
    Serial.print("PLL ");
    Serial.print(pll);
    Serial.print(" actual frequency: ");
    Serial.println(actual_freq);
    Serial.print("Target PLL frequency: ");
    Serial.println(pll_freq);
    Serial.print("Error: ");
    Serial.println(pll_freq - actual_freq);
    Serial.print("Feedback = ");
    Serial.print(int_mult);
    Serial.print(" + ");
    Serial.print(ac);
    Serial.print("/");
    Serial.println(bd);
  }
}


// Tune the receiver to the specified frequency
void tune_freq(uint32_t freq)
{
  uint32_t pll_freq, tolerance;

  pll_freq = 6*(freq - IF_FREQ); // Using multisynth integer divide by 6

  // Calculate the requirement on the error of the approximation to get 
  // less than MAX_TUNING_ERROR Hz of resulting error when the multisynth divides by 6.
  tolerance = 6*MAX_TUNING_ERROR; 

  tune_pll(pll_freq, tolerance, SI5351_PLL_A);
}


// Set up and power on the frequency test sweep outputs.
void enable_test_source(uint32_t freq)
{
  si5351.powerDownOutput(1, false);
  si5351.powerDownOutput(2, false);
  si5351.enableOutput(1, true);
  si5351.enableOutput(2, true);
  si5351.outputDrive(1, SI5351_DRIVE_2MA);
  si5351.outputDrive(2, SI5351_DRIVE_2MA);
  si5351.invertOutput(2, true);
  si5351.setupMultisynthInt(1, SI5351_PLL_B, SI5351_MULTISYNTH_DIV_6);
  si5351.setupMultisynthInt(2, SI5351_PLL_B, SI5351_MULTISYNTH_DIV_6);
  tune_pll(6*freq, 6*MAX_TUNING_ERROR, SI5351_PLL_B); // The PLL operates at 6 times the output frequency
  si5351.resetPLL(SI5351_PLL_B);
  delay(PLL_DELAY);
}


// Power down the frequency test sweep outputs.
void disable_test_source()
{
  si5351.enableOutput(1, false);
  si5351.enableOutput(2, false);
  si5351.powerDownOutput(1, true);
  si5351.powerDownOutput(2, true);  
}


// Return true iff f is within the allowed range of frequencies
bool is_valid_freq(uint32_t f)
{
  return (f >= MIN_TUNE_FREQ) && (f <= MAX_TUNE_FREQ);
}


bool is_valid_freq(uint64_t f)
{
  return (f >= MIN_TUNE_FREQ) && (f <= MAX_TUNE_FREQ);
}


void isr_a(void)
{
  rot_enc.isr_rot_a_change();
}


void isr_b(void)
{
  rot_enc.isr_rot_b_change();
}
