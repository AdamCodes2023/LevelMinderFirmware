#include "LevelSensorV1.h"

#define INTEGER_PLACE_SHIFT 100.0 // Shift integer value to display decimal places

// Create display instance with I2C address 0x70 (default for HT16K33)
HT16K33 display(DISPLAY_I2C_ADDRESS);

void initializeDisplay() {
  // Initialize the display
  if (!display.begin()) {
    Serial.println("Failed to initialize display!");
  }

  // Set brightness (0-15)
  display.setBrightness(10);

  // Clear the display
  display.displayClear();
}

void displayOn() {
  // Turn on the display
  display.displayOn();
}

void displayOff() {
  // Turn off the display
  display.displayOff();
}

void displayIntLong(long x) {
  // Calculate the display adjustment factor based on the unit of measurement
  if (!isUnitMeters && !boardTestMode) {
    x = (long)(x * metersToFeetConversionFactor);
  }

  displayOn();

  // Display the integer value on the HT16K33
  display.displayFloat((float)x / INTEGER_PLACE_SHIFT, 2); // Display as a float with 2 decimal places
}

void displayProbeMissingError() {
  // Define raw 7-segment patterns for BLANK, 'L', 'O', 'S'
  uint8_t loss[4] = {
    0x00,                                           // Blank
    SEG_F | SEG_E | SEG_D,                          // 'L'
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,  // 'O'
    SEG_A | SEG_F | SEG_G | SEG_C | SEG_D           // 'S'
  };

  displayOn();

  // Send the 4-byte array to the display (false = colon OFF)
  display.displayRaw(loss, false);
}

/*
void displayOn() {
  // turns on display
  digitalWrite(displayEnablePin, LOW);
  shiftOut(displayDataPin, displayClockPin, MSBFIRST, B00000001);
  digitalWrite(displayEnablePin, HIGH);
  delay(10);
}

void displayOff() {
  // turns off display
  digitalWrite(displayEnablePin, LOW);
  shiftOut(displayDataPin, displayClockPin, MSBFIRST, B00000000);
  digitalWrite(displayEnablePin, HIGH);
  delay(10);
}

void displayTest1() {
  displayOn();

  // displays 5.4321
  digitalWrite(displayEnablePin, LOW); // send 3 bytes to display register. See data sheet page 9
  // you can also insert decimal or hexadecimal numbers in place of the binary numbers
  // we're using binary as you can easily match the nibbles (4-bits) against the table
  // in data sheet page 8
  shiftOut(displayDataPin, displayClockPin, MSBFIRST, B11010101); // D23~D16
  shiftOut(displayDataPin, displayClockPin, MSBFIRST, B01000011); // D15~D8
  shiftOut(displayDataPin, displayClockPin, MSBFIRST, B00100001); // D7~D0
  digitalWrite(displayEnablePin, HIGH);
  delay(10);
}

void displayTest2() {
  displayOn();

  // displays ABCDE
  digitalWrite(displayEnablePin, LOW); // send 3 bytes to display register. See data sheet page 9
  // you can also insert decimal or hexadecimal numbers in place of the binary numbers
  // we're using binary as you can easily match the nibbles (4-bits) against the table
  // in data sheet page 8
  shiftOut(displayDataPin, displayClockPin, MSBFIRST, B10001010); // D23~D16
  shiftOut(displayDataPin, displayClockPin, MSBFIRST, B10111100); // D15~D8
  shiftOut(displayDataPin, displayClockPin, MSBFIRST, B11011110); // D7~D0
  digitalWrite(displayEnablePin, HIGH);
  delay(10);
}

void displayIntLong(long x) {
  if (!isUnitMeters && !boardTestMode) {
    x = (long)(x * metersToFeetConversionFactor);
  }

  displayOn();

  // takes a long between 0~99999 and sends it to the MC14489
  int numbers[5];
  byte a=0; 
  byte b=0; 
  byte c=0; // will hold the three bytes to send to the MC14489 

  // first split the incoming long into five separate digits
  numbers[4] = int ( x / 1000 ); // right-most digit (will be BANK5)
  x = x % 1000; 
  numbers[3] = int ( x / 100 );
  x = x % 100; 
  numbers[2] = int ( x / 10 );
  x = x % 10; 
  numbers[1] = x % 10;
  x = 0; 
  numbers[0] = x; // left-most digit (will be BANK1)

  // now to create the three bytes to send to the MC14489
  // build byte c which holds digits 1 and 2
  c = numbers[3];
  c = c << 4; // move the nibble to the left
  c = c | numbers[4];
  // build byte b which holds digits 3 and 4
  b = numbers [1];
  b = b << 4;
  b = b | numbers[2];
  // build byte a which holds the brightness bit, decimal points and digit 5
  a = B10100000 | numbers[0]; // full brightness, decimal point in BANK2

  // now send the bytes to the MC14489
  digitalWrite(displayEnablePin, LOW);
  shiftOut(displayDataPin, displayClockPin, MSBFIRST, a);
  shiftOut(displayDataPin, displayClockPin, MSBFIRST, b);
  shiftOut(displayDataPin, displayClockPin, MSBFIRST, c); 
  digitalWrite(displayEnablePin, HIGH);
  delay(10); 
}

void displayProbeMissingError() {
  // configures display for special characters
  digitalWrite(displayEnablePin, LOW); // send 1 byte to display register. See data sheet page 9
  // you can also insert decimal or hexadecimal numbers in place of the binary numbers
  // we're using binary as you can easily match the nibbles (4-bits) against the table
  // in data sheet page 8
  shiftOut(displayDataPin, displayClockPin, MSBFIRST, B01000111); // C7~C0
  digitalWrite(displayEnablePin, HIGH);
  delay(10);

  // displays PrbE
  digitalWrite(displayEnablePin, LOW); // send 3 bytes to display register. See data sheet page 9
  // you can also insert decimal or hexadecimal numbers in place of the binary numbers
  // we're using binary as you can easily match the nibbles (4-bits) against the table
  // in data sheet page 8
  shiftOut(displayDataPin, displayClockPin, MSBFIRST, B10000000); // D23~D16
  shiftOut(displayDataPin, displayClockPin, MSBFIRST, B11101011); // D15~D8
  shiftOut(displayDataPin, displayClockPin, MSBFIRST, B10011000); // D7~D0
  digitalWrite(displayEnablePin, HIGH);
  delay(10);
}
*/