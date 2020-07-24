  /*
 * Arduino based Hex Keypad digital input with an
 * MCP23008 used for the Control lines for a 
 * 1802 Membership card.
 * 
 * Copyright (c) 2020 by Gaston Williams
 * 
 * Based on the 1802 Membership card hardware by Lee Hart. 
 * The 1802 Membership card is available here: 
 * http://www.sunrise-ev.com/1802.htm 
 * 
 * The 1802 Membership Card Microcomputer 
 * Copyright (c) 2006-2020  by Lee A. Hart.
 * 
 * A Sparkfun 4x4 Keypad was used for key input.  The star
 * key * is mapped to E and the hash key # is F.
 * 
 * The Hex Keypad Arduino Library is based upon the 
 * Sparkfun Qwiic Keypad Arduino Library modified for
 * hexadecimal input from a 4x4 Keypad.
 * 
 * See the Hex Keypad Arduino Library Github project for 
 * information, firmware and an Arduino library for this 
 * hexadecimal keypad.
 * 
 * The Hex Keypad Arduino Library is available here:
 * https://github.com/fourstix/Hex_Keypad_Arduino_Library
 * 
 * 4x4 button Keypad is available here:
 * https://www.adafruit.com/product/3844
 * https://www.sparkfun.com/products/retired/14881
 * 
 * A 16 x 2 LCD is used for the display.  This code 
 * has been tested with the Sparkfun SERLCD display
 * and a generic I2C based 1602 LCD display.
 * 
 * The Sparkfun SerLCD is available here:
 * https://www.sparkfun.com/products/14073
 * 
 * All libraries and hardware designs are copyright their respective authors.
 * 
 * Sparkfun Qwiic Keypad Arduino Library
 * Copyright (c) 2019 SparkFun Electronics
 * Written by Pete Lewis @ SparkFun Electronics, 3/12/2019
 * 
 * Adafruit MCP23008 GPIO Expander Library
 * Copyright (c) 2012-2019 Adadruit Industries
 * Written by Limor Fried/Ladyada for Adafruit Industries.  
 * 
 * The Arduino-MCP23017 library
 * Copyright (c) 2017 Bertrand Lemasle
 * 
 * The Hex_Keypad_Arduino_Library
 * Copyright (c) 2020 by Gaston Williams
 * 
 * Sparkfun SerLCD Arduino Library
 * Written by Gaston Williams and Nathan Seidle @ SparkFun
 * Copyright (c) 2018 Sparkfun Electronics
 * 
 * LiquidCryistal_I2C Arduino Library
 * Written by Frank de Brabander
 * Copyright (c) 2011 DFRobot.com 
 * 
 * The 1802 Membership Card Microcomputer hardware design
 * Copyright (c) 2006-2020 by Lee A. Hart
 * 
 * Many thanks to the original authors for making their designs and code avaialble.
 */
#include <Wire.h> 
#include "Hex_Keypad_Arduino_Library.h"
#include "Adafruit_MCP23008.h"
#include "MCP23017.h" 
//Using Sparkfun Serial LCD
//#include <SerLCD.h>
//Using Generic 16 x 2 LCD Backpack
#include <LiquidCrystal_I2C.h>

//Change debug token from 0 to 1 to include debug code in compile
#define DEBUG 0

//Offset for Control MCP23008 GPIO Expander Address 0x20
#define MCP_CTRL  0

//Address for Data MCP23017 GPIO Expander
#define MCP_DATA 0x21

//Address for Address MCP23017 GPIO Expander
#define MCP_DATA 0x21

//Define mask for data byte
#define BYTE_MASK  0x00FF   

//Defines for setting GPIO Control bits
#define WAIT_BIT  0x01
#define CLEAR_BIT 0x02
#define WR_BIT    0x04
#define EF4_BIT   0x08
//EF3 is used for Serial input (Pin 9)
#define EF2_BIT   0x10
//EF1 is used by Pixie Video

//Last two bits are status bits
#define STATUS_MASK 0x03

//Ignore /EF4 control line
#define CONTROL_MASK 0x17

//Values for control bit states
#define STATUS_LOAD  0
#define STATUS_RESET 1
#define STATUS_WAIT  2
#define STATUS_RUN   3

//Hold ef4 low for minimum of 50 mSec to simulate keypress
#define KEY_PRESS_DURATION 50 

//Minimum time for rapid input
#define RAPID_INPUT_DURATION  10

//Brief wait before next input 
#define COMMAND_DELAY 50

/*     
 * Analog pin for Digital Input    
 * Arduino Pin        Keypad
 *      A0            /RDY from hex Keypad
 */
//Pin to poll for Keypad input
#define KEY_READY_PIN A0

/*
 * Button input pins
 * Arduino Pin            Button
 *      D7              Run Button
 *      D6              Load Button
 *      D5              M/P Button
 *      D4              Input Button
 *      D3                  --      
 *      D2                  --  
 *      TX (Reserved)       --
 *      RX (Reserved)       --
 */  
//Pins to poll for Control Buttons
#define RUN_BTN_PIN   7
#define LOAD_BTN_PIN  6
#define MP_BTN_PIN    5
#define INPUT_BTN_PIN A1

//Number of state buttons (Run, Load, M/P)
#define STATE_BTN_COUNT 3

//Indexes for button arraya
#define RUN_IDX  0
#define LOAD_IDX 1
#define MP_IDX   2

//Space character used for invalid key input
#define NO_KEY_CHAR ' '

//Page size for loading programs
#define PAGE_SIZE 256

//Size of a Byte written in ASCII characters
#define BYTE_CHAR_SIZE 2

//allows input button to continuously hold input 
boolean hold_input = false; 

//data written to 1802 data in lines on MCP23017 Port A
byte data_in = 0x00;
//previous data written to MCP23017 Port A
byte old_data_in = 0x00;

//data read from 1802 data out lines on MCP23017 Port B
byte data_out = 0x00;
//previous data read from MCP23017 Port B
byte old_data_out = 0x00;

//Q Bit read from RX_PIN (With inversion)
boolean q_bit = false;
//previous Q bit read from RX_PIN
boolean old_q_bit = false;

//EF2 flag goes LOW when asserted
boolean assert_ef2 = false;

//byte for control lines
byte control_data = 0x00;

//previous control line data
byte old_control_data = 0x00;

//buffer for characters typed on the keypad
uint16_t key_buffer = 0x00;

//External Flag 4 staus used for keyboard input key
boolean input_pressed = false;

//pin numbers for buttons
int btn_pins[3] = {RUN_BTN_PIN, LOAD_BTN_PIN, MP_BTN_PIN};

//flags for button status
boolean btn_states[3] = {false, false, false};

//flags for previous state
boolean last_btn_states[3] = {false, false, false};

//time when input key was pressed
unsigned long t_input_down;

//time when button changed
unsigned long t_debounce_times[3];

//character from keypad
char c_input = NO_KEY_CHAR;


/*
 * Serial Data Lines for Membershp Card 
 * Arduino Pin        1802 Membership Card
 *      D9                    /EF3
 *      D8                    Q
 */
//Pins for Serial communication
#define TX_PIN 9 
#define RX_PIN 8

//Allow character input from serial
boolean allow_serial = true;

//Terminal mode for serial communication
boolean terminal_mode = false;

//Flag to show data in terminal mode
boolean show_data = false;

//Define Sparkfun Qwiic Keypad
KEYPAD hexKeypad; 

//Define MCP23008 port
Adafruit_MCP23008 mcpCtrl;


//Define MCP23017 
MCP23017 mcpData = MCP23017(MCP_DATA);


// initialize the SerLCD with default i2c address 0x72
//SerLCD lcd;
// LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27,16,2);

/*
 * Keypad utilities 
 */
//Check the Keypad int pin and returns true
//if a key is ready to be read from the keypad
boolean keyPressed() {
  //Ready line goes low when a key is pressed
  return !digitalRead(KEY_READY_PIN);
}

//Check the keypad and get a charcter if button was pressed
char checkKeypad() {
  //Set character to invalid value
  char button = NO_KEY_CHAR;

  // Check ready line and get the last key pressed
  if (keyPressed()) {
    //Put key press in buffer and read it
    hexKeypad.updateFIFO();
    button = hexKeypad.getButton();
    
    if (button <= 0) {
      //for anything else return a space
      button = NO_KEY_CHAR;
    } // if-else button > 0
  }  // if keyPressed()
  return button;
} // checkKeypad

/*
 * Character utilities 
 */   
// Pretty print two hex digits for display
void print2Hex(uint8_t v) {  
  //If single Hex digit
  if (v < 0x10) {
   Serial.print(F("0"));
  } // if v < 0x10
  Serial.print(v, HEX);
}

//Get the numeric value of a hexadecimal character
byte getHexValue(char d) {
  byte value = 0x00;
  // check to see what range value is in
  if (isDigit(d)) {
    value = d - '0';   
  } else {
      value = 10 + toupper(d) - 'A';
  } // if else
  return value;
} // getHexValue

//Process a character as a hex digit 
void processHexChar(char h) {
  //shift previous digit into high nibble and clear rest of bits
  key_buffer = (key_buffer << 4) & 0x00F0;
  key_buffer |= getHexValue(h);

  //set data value for mcp23017
  data_in = key_buffer & BYTE_MASK;
  
  #if DEBUG
    Serial.print(F("Key Buffer: "));
    print2Hex(data_in);
    Serial.println();
  #endif      
} //processHexChar


// Check for serial input 
char checkSerial () {
  char cs = NO_KEY_CHAR;  
     if (Serial.available()) {
    // read a character   
    cs = Serial.read();
    } // if Serial.available
    return cs;
} // checkSerial

//Process a character from the keypad        
void processChar(char c) {
  //Process a character from either the keypad or serial monitor
  switch(c) {        
    //Input
    case '#':
    case ',':
    case 'I':
    case 'i':
      //Set ef4 true
      input_pressed = true;
      //Set timestamp for holding flag for duration of keypress
      t_input_down = millis();  
      #if DEBUG
        Serial.println(F("Input"));           
      #endif
    break;

    //Toggle the /EF2 flag
    case 'o':
    case 'O':
      assert_ef2 = !assert_ef2;
      Serial.print("\EF2 flag is ");
      if(assert_ef2) {
        Serial.println("on.");
      } else {
        Serial.println("off.");
      } //if-elase assert_ef2
    break;

    //Start the serial terminal mode
    case 'T':
    case 't':
      terminal_mode = true;
      setupTerminal();
    break;  

    //Show menu
    case '?':
      showMenu();
    break;   
      
    //Process Hex characters
    default:         
      if (isHexadecimalDigit(c)) {
        #if DEBUG
          Serial.println(c);                  
        #endif
        processHexChar(c);
      }
       break;
  } // switch    
} //processChar

/*
 * Button utilities 
 */

//Read the state of the button with debouncing
void readButtonState(byte btn_idx) {
  int btn_pin    = btn_pins[btn_idx];
  int btn_state  = btn_states[btn_idx];
  int last_state = last_btn_states[btn_idx];
   
  // read the state of the switch into a local variable:
  int reading    = digitalRead(btn_pin);  
  // get the time since last change
  unsigned long last_debounce_time = t_debounce_times[btn_idx];

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != last_state) {
    // reset the debouncing timer
    last_debounce_time = millis();
    #if DEBUG
      Serial.print("Pin ");
      Serial.print(btn_pin);
      Serial.print(" Last State: ");
      Serial.print(last_state);
      Serial.print(" Reading: ");
      Serial.println(reading);
    #endif
  }

  if ((millis() - last_debounce_time) > KEY_PRESS_DURATION) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != btn_state) {
      btn_states[btn_idx] = reading;
    } //if reading != btn_state    
  } // time since  > KEY_PRESS_DELAY
  
  //Save the reading as last state for next check
  last_btn_states[btn_idx] = reading;
  //Save the debounce time for next check
  t_debounce_times[btn_idx] = last_debounce_time;
} //readButtonState



//Set the state of all buttons
void setButtonStates() {
  for (int i=0; i < STATE_BTN_COUNT; i++) {
    readButtonState(i);
  } //for
} //setButtonStates

//Get the control flags based on button status  
byte getControlFlags() {
  byte control_flags = 0x00;
  //Set the flags LOW means button is pressed
  boolean run_flag = btn_states[RUN_IDX] == LOW;
  boolean load_flag = btn_states[LOAD_IDX] == LOW;
  boolean mp_flag = btn_states[MP_IDX] == LOW;

  //If run is on, set /CLEAR hgh
  if (run_flag) {
    control_flags |= CLEAR_BIT;
  }

  //If load is off, set /WAIT high
  if (!load_flag) {
    control_flags |= WAIT_BIT;    
  }

  //If M/P is not down, set the write bit on
  if (!mp_flag) {
    control_flags |= WR_BIT;
  }

  //Set the /EF2 input
  if (assert_ef2) {
    //Negative Logic: /EF2 is LOW when TRUE
    control_flags &= ~EF2_BIT;
  } else {
    //Negative Logic: /EF2 is HIGH when FALSE
    control_flags |= EF2_BIT;
  } //if-else assert_ef2
  return control_flags;
} //getControlFlags

//indicates status information has changed for display update
boolean statusChanged() {
  //Always show control information changes
  boolean changed = 
    //ignore changes in the Input bit (/EF4)
    ((control_data & CONTROL_MASK) != (old_control_data & CONTROL_MASK));
    

  //In terminal mode, ignore Q bit changes and data changes 
  //unless show data flag is on
  if (!terminal_mode || show_data) {
    changed = changed || (data_out != old_data_out);
    changed = changed || (q_bit != old_q_bit);
  } //if !terminal_mode || show_data

  return changed;
} //statusChanged
/*
 * Menu and Display functions
 */
 // Show Menu of commands on Serial monitor
void showMenu() {
  Serial.println(F(" 0-F = hex digit for code input"));
  Serial.println(F(" #,I = Input data to memory during load mode"));
  Serial.println(F("     - Multi-byte input load eg: 00,01,02,03#"));  
  Serial.println(F("     - Simulate an Input button press in run mode")); 
  Serial.println(F(" O - toggle state of Output control line /EF2"));
  Serial.println(F(" T - enable serial Terminal."));  
  Serial.println(F(" ? = show this menu")); 
}

//print control and data information to the display
void printStatus(boolean force = false) {
  //force always clears the screen
  if (force) {
    lcd.clear();
  }
  //if information changed, update display
  if (statusChanged() || force) {
    //Mask off status and mem_write bits
    byte status_1802 = control_data & STATUS_MASK;
    byte mem_write = control_data & WR_BIT;
    
    lcd.setCursor(0,0);
    
    //print the current status to display
    switch (status_1802) {
      case STATUS_LOAD:
        lcd.print(F("Load  "));
      break;
      
      case STATUS_WAIT:
        lcd.print(F("Wait  "));
      break;
      
      case STATUS_RESET:
        lcd.clear(); //clear display
        lcd.print(F("Reset "));
      break;
      
      case STATUS_RUN:
        lcd.print(F("Run   "));
      break;
      
      //Unknown
      default:
        lcd.print(F("????? "));
      break;
    } //switch

    //print status of memory protection
    if (mem_write) {
      lcd.print(F("W"));  //Write Enabled
    } else {
      lcd.print(F("P"));  //Protected
    } //if else !mem_prot   

    if (assert_ef2) {
      lcd.print(F(" !2"));  //Output Line /EF2 on
    } else {
      lcd.print(F("   "));  //Output Line /EF2 off
    } //if-else asset_ef2

    //print the data out on next line
    if (status_1802 != STATUS_RESET) {
      lcd.setCursor(0, 1);
      if(!terminal_mode || show_data) {
        //Show Data bus value
        if (data_out < 0x10) {
          //If single Hex digit, print leading zero
          lcd.print(F("0"));
         } // if data_out < 0x10
         lcd.print(data_out, HEX);
         
         //Show Q bit status
         if (q_bit) {
          //Show a T to indicate terminal mode
          if (terminal_mode) {
            lcd.print(F(" T"));        
          } else {
            lcd.print(F(" Q"));        
          } //if-else terminal mode
        } else {
          lcd.print(F("  "));
        } //if-else q_bit
      } else {
        lcd.print(F("Terminal"));        
      }
    }  // if !STATUS_RESET
  } //if statusChanged
} //printStatus


/*
 *    Digital Inputs for Control buttons
 * Arduino Pin            Digital Input
 *      D2                    --
 *      D3                    --
 *      D4                    --
 *      D5                M/P Button
 *      D6                Load Button
 *      D7                Run Button
 *      
 *    Serial Pins for communication     
 * Arduino Pin        1802 Membership Card
 *      D8 (TX)             /EF3
 *      D9 (RX)             TX (/Q)
 *      D10                 --    
 *      D11                 --    
 *      D12                 --    
 *      D13 (Output)        --
 *      
 *    Analog pin for Digital Input
 *      A0                  /RDY from Hex Keypad
 *      A1                  Input Button
 *      
 *    I2C pins for Qwiic Keypad  
 *      A4 (SDA)              
 *      A5 (SCL)              
 *      
 */
 
void setup() {
  //Set up MCP23017
  Wire.begin(); 
  mcpData.init();
  mcpData.portMode(MCP23017_PORT::A, 0);         //Port A as ouput
  mcpData.portMode(MCP23017_PORT::B, 0b11111111);//Port B as input

  //Initialize GPIO ports
  mcpData.writeRegister(MCP23017_REGISTER::GPIOA, 0x00);
  mcpData.writeRegister(MCP23017_REGISTER::GPIOB, 0x00);
    
  //Set up the Qwiic Keypad communication
  hexKeypad.begin();
  
  //Set up digital input for keypad
  pinMode(KEY_READY_PIN, INPUT);
  
  //Set up MCP23008 for control lines
  mcpCtrl.begin(MCP_CTRL);

  //Set all the GPIO pins to outputs
  for (int i = 0; i < 8; i++) {
    mcpCtrl.pinMode(i, OUTPUT);
  }//for
   //Set up RX pin for /Q
   pinMode(RX_PIN, INPUT_PULLUP);
 
  //Set up button inputs
  pinMode(INPUT_BTN_PIN, INPUT_PULLUP);  
  pinMode(MP_BTN_PIN, INPUT_PULLUP);
  pinMode(LOAD_BTN_PIN, INPUT_PULLUP);
  pinMode(RUN_BTN_PIN, INPUT_PULLUP);

  //Initialize serial for input and debugging
  Serial.begin(115200);

  //Setup Serlcd display
//  lcd.begin(Wire);
//  lcd.clear();
  
  //Set up LCD display
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  //Show menu of commands
  showMenu();
} //setup

void loop() {
  //Set the state of the control buttons
  setButtonStates();
  //Check the input button
  hold_input = !digitalRead(INPUT_BTN_PIN);
  
  if (hold_input) {
    t_input_down = millis();
    #if DEBUG
      //Print debug message only once
      if (!input_pressed) {
        Serial.println("Input Button down!");
      } //if !input_pressed
    #endif
    input_pressed = true;
    allow_serial = true;
  } else if (input_pressed && (millis() - t_input_down < KEY_PRESS_DURATION)) {
    //Continue to hold input key for duration
    input_pressed = true;
    //block input during input keystroke time to avoid Serial overrun
    allow_serial = false;
  } else {
    input_pressed = false;
    allow_serial = true;
  } // if hold_input else if
  
  //Set up control data
  control_data = getControlFlags();
  
  //Set /EF4 bit if needed (Negative Logic: 1 = not pressed)
  if (!input_pressed) {
    control_data |= EF4_BIT;
  } //if !input_pressed

  //write out conrol data to MCP23008
  mcpCtrl.writeGPIO(control_data);
    
  #if DEBUG
    if (control_data != old_control_data) {
      Serial.print(F("Control Data: "));
      Serial.println(control_data, HEX);
    }
  #endif
  
  //If the key data has changed write to 1802
  if (data_in != old_data_in) {
    #if DEBUG
      Serial.print("Writing ");
      print2Hex(data_in);
      Serial.println(" to data bus.");
    #endif
    mcpData.writeRegister(MCP23017_REGISTER::GPIOA, data_in);
    //Save key data after sending out
    old_data_in = data_in;
    //wait a bit after sending
    delay(20);
  } //if data_in != old_data_in  
  
  //get next character from keypad
  c_input = checkKeypad();
  
  //Serial Terminal over-rides serial input
  if (terminal_mode) {
    doTerminal();
    //When clear bit goes low, we entered Load or Reset
    //So end the terminal session
    if (!(control_data & CLEAR_BIT)) {
      endTerminal();
    } //if control_data CLEAR_BIT low 
  //Else if we don't have a character check the Serial input
  } else if (isSpace(c_input) && allow_serial) {
     c_input = checkSerial();
  } //if terminal - else if isSpace && allow_serial
  
  //process any character input
  processChar(c_input);  

 //Save previous data before reading
  old_data_out = data_out;
  
  //Read the input data
  data_out = mcpData.readPort(MCP23017_PORT::B);

  //Save previous q value before reading
  old_q_bit = q_bit;
  //RX is the opposite sense of Q
  q_bit = !digitalRead(RX_PIN);

  #if DEBUG
    if (data_out != old_data_out) {
      Serial.println();
      Serial.print(F("Data Out: "));
      print2Hex(data_out);
      Serial.println();
    } // if data_out != old_data_out
    if (q_bit != old_q_bit) {
      Serial.println();
      Serial.print(F("Q: "));
      print2Hex(q_bit);
      Serial.println();
    } // if q_bit != old_q_bit
  #endif  
  
  printStatus();
   
  //save control values after display
  old_control_data = control_data;
} //loop
