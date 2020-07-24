/*
  Serial Terminal Mode for 1802 Membership Card

  Arduino Pin     MCard1802
    9 (TX)        /EF3 (RX)   P1 - 27 
    8 (RX)          /Q  (TX)  (Inverted from P1-12)
    GND             GND       P1 -  1 (or P1 - 30)
*/

//Note: SoftwareSerial is *NOT* Full-duplex and won't work here.
#include <AltSoftSerial.h>

//Mask for 7-bit ASCII
#define ASCII_MASK    0x7F

// AltSoftSerial always uses these pins:
//
// Board          Transmit  Receive   PWM Unusable
// -----          --------  -------   ------------
// Teensy 3.0 & 3.1  21        20         22
// Teensy 2.0         9        10       (none)
// Teensy++ 2.0      25         4       26, 27
// Arduino Uno        9         8         10
// Arduino Leonardo   5        13       (none)
// Arduino Mega      46        48       44, 45
// Wiring-S           5         6          4
// Sanguino          13        14         12


//RX pin 8, TX pin 9, with RX and TX inversion
AltSoftSerial altSerial(true);
//Default is RX pin 8, TX pin 9, no inversion
//AltSoftSerial altSerial;

//Variables for Ascii terminal
boolean escape        = false;
boolean hex_escape    = false;
char    hex_char      = 0x00;
int     baud          = 1200;

void setupTerminal() {
  //1200 baud seems to work best
  altSerial.begin(baud);
  showTerminalMenu();
  Serial.println("Ready!");
  //Turn data display off
  show_data = false;
  //Update display
  printStatus(true);

  //Establish communication with <CR>
  sendChar('\r'); 
}

//Process serial input and output in the terminal
void doTerminal() {
  // If anything comes in Serial (USB),
  if (Serial.available()) {      
    // get the new byte:
    char nextChar = (char)Serial.read();            
    processTerminalChar(nextChar);     
  }
  if (altSerial.available()) {
    // get the new byte:
    char inChar = (char) altSerial.read();

    //Mask off high bit
    inChar = (inChar & 0x7F);

   Serial.write(inChar);
  } // if altSerial.available
} //doTerminal

//End terminal session with message
void endTerminal() {
  terminal_mode = false;
  show_data = false;
  Serial.println();
  Serial.println(F("*** ASCII Terminal End ***"));
  //Update the display
  printStatus(true);
}

//Process characters with escapes in terminal mode
void processTerminalChar(char inChar) {
  //First check to see if this is the second hex digit in \hh escape
  if (hex_escape) {
    processHexEscape(inChar);    
    //Hex Escape Sequnce done, reset variables
    hex_escape = false;
    hex_char = 0x00;
  } else if (escape) { //Next chek to see if we are in an escape squence
    processEscape(inChar);
    //Finished process escape (hex escape flag may still be true);  
    escape = false;
  } else if (inChar == '\\') { //Check for start of a new escape squence
    //Next character determines what kind of escape sequence
    escape = true;
  } else { //Regular character
    // Send it out to the 1802 on pins 8 & 9
    sendChar(inChar);   
  }//if-else hex_escape
} // processTerminalChar

//Process characters in an escape sequence
void processEscape(char e_char) {
  switch(e_char) {
    //Backslash
    case '\\':
      sendChar('\\');   
    break;
    
    //Show menu
    case '?':      
      showTerminalMenu();
    break;
    
    //Quit
    case 'q':      
    case 'Q':      
      if (confirmQuit()) {
        endTerminal();
      } // if confirmQuit
    break;

    //New Line
    case 'n':
    case 'N':
      sendChar('\n');   
    break;
          
    //Carriage Return
    case 'r':
    case 'R':
      sendChar('\r');   
    break;
    
    //Tab
    case 't':
    case 'T':
      sendChar('\t');   
    break;
     
    //Escape (ESC)
    case 'e':
    case 'E':
      sendChar(0x1b);   
    break; 

    //Toggle Output data display
    case 'd':
    case 'D':
      show_data = !show_data;
      //Force update to the display
      printStatus(true);
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
    
    default: 
       //Hexadecimal escape sequence \h or \hh
       if (isHexadecimalDigit(e_char)) {
         //Hex escape sequence can be one or two hex characters
         hex_escape = true;
         //Set lower nibble to first hex digit
         hex_char = getHexValue(e_char);            
       } else {
        //Unknown character after escape, send backslash plus character as literal
        sendChar('\\');
        sendChar(e_char);  
       } //if-else isHexadecimalDigit
    break;      
  } // switch
} //processEscape

void processHexEscape(char h_char) {
  if (isHexadecimalDigit(h_char)) {
    //Shift first hex digit into high nibble
    hex_char = hex_char << 4;
    //Put second hex digit into low nibble
    hex_char |= getHexValue(h_char);
    //Mask off 8th bit, to ensure valid ASCII
    hex_char &= ASCII_MASK;
    sendChar(hex_char);
  } else {  //Non hex character, so send hex code we have so far
    sendChar(hex_char);
    
    //Check to see if next char starts another escape sequence
    if (h_char == '\\') {
      escape = true;
    } else { //Otherwise just send next character
      sendChar(h_char);
    } // if-else h_char is backslash      
  } //if=else isHexadecimalDigit  
} //processHexEscape


//Send a character to the Membership Card
void sendChar(char c_send) {
  //Send the character out on pins 8 and 9
  altSerial.write(c_send);   
  //wait a bit for 1802 to process character sent
  delay(20);  
}

//Show the ASCII Terminal menu
void showTerminalMenu() {
  Serial.println();
  Serial.println(F("**** ASCII Terminal ****"));
  Serial.println(F("Enter '\\q' to quit terminal."));
  Serial.println(F("Escape characters:"));
  Serial.println(F("\\t    tab (09)"));
  Serial.println(F("\\r    carriage return (0D)"));
  Serial.println(F("\\n    new line (0A)"));
  Serial.println(F("\\e    escape (1B), break or cancel command"));
  Serial.println(F("\\\\    backslash (5C)"));
  Serial.println(F("\\hh   hexadecimal ASCII character, 00 to 7F"));
  Serial.println(F("\\d    toggle Data to display"));
  Serial.println(F("\\o    toggle state of Output control line /EF2"));
  Serial.println(F("\\?    print this menu"));
  Serial.println(F("\\q    quit terminal mode (\\qy = quit immediately)"));
  Serial.println();
}

//Confirm the choce to quit
/*
  Confirm the chosen action by asking for a yes / no
*/
boolean confirmQuit() {
    bool result = false;
    Serial.println();
    Serial.print(F("Quit ASCII Terminal. "));
    Serial.println(F("Are you sure (y/n)?"));
    while (true) {
      while (! Serial.available());  // wait for characters
      char c = Serial.read();  // read a character 
      if (isAlpha(c)) {   
          if (c == 'y' || c == 'Y') {
            result = true;
          } // if c == y
          break;  // break out of reading loop
      }  // if isAlpha
    } // while reading characters
  if (!result) {
    Serial.println(F("Canceled."));
  }  
  return result;
}  // confirmQuit
