//LEVEL SENSOR TEST PROGRAM
#include "LevelSensorV1.h"

//#include <SPI.h>                                  		//-- moved to arduinoGlue.h
#include "Arduino.h"

//#include <BLEDevice.h>                            		//-- moved to arduinoGlue.h
//#include <BLEUtils.h>                             		//-- moved to arduinoGlue.h
//#include <BLEServer.h>                            		//-- moved to arduinoGlue.h

// github link: https://github.com/4-20ma/ModbusMaster
//#include <ModbusMaster.h>                         		//-- moved to arduinoGlue.h

//#include <Preferences.h>                          		//-- moved to arduinoGlue.h

//#include <Wire.h>                                 		//-- moved to arduinoGlue.h
//#include <AD56X2.h>                               		//-- moved to arduinoGlue.h

// GLOBAL VARIABLES AND CONSTANTS
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

	//-- moved to arduinoGlue.h // #define SEND_CONFIG_SERVICE_UUID            "043d47db-07b6-4112-9505-b8dd9dcd3851"
	//-- moved to arduinoGlue.h // #define SEND_CONFIG_CHARACTERISTIC_UUID     "ea324d87-4867-4ea5-babb-39f1a967bb66"
	//-- moved to arduinoGlue.h // #define DISPLAY_CONFIG_SERVICE_UUID         "4798e66a-1287-48b0-8096-aed974e68b27"
	//-- moved to arduinoGlue.h // #define DISPLAY_CONFIG_CHARACTERISTIC_UUID  "9994a725-36d2-4151-b4fb-204ac9b53ed8"
	//-- moved to arduinoGlue.h // #define SEND_SIMULATION_SERVICE_UUID        "807e4f7a-761d-44f2-9ed9-65b9b97b4abb"
	//-- moved to arduinoGlue.h // #define SEND_SIMULATION_CHARACTERISTIC_UUID "d5dd0354-7334-4ab1-9546-64243e4ebaef"

BLECharacteristic *pDisplayConfigCharacteristic = NULL;
BLECharacteristic *pUpdateProgressCharacteristic = NULL;
bool bluetoothActive = false;

// OTA update variables
bool fwWriteFlag = false;
bool fwUpdateFlag = false;
bool otaInProgress = false;
uint32_t otaFileSize = 0;
uint32_t otaReceived = 0;
int lastProgressPercent = -1;

bool simulationMode = false;
bool testOverride = false;
bool silenceOverride = false;
bool bluetoothResetButtonPressed = false;
bool alarmTestButtonPressed = false;
bool alarmSilenceButtonPressed = false;

bool ledAlarmOn = true;

int bluetoothResetPin = 0;

int relayOnePin = 4;
int relayTwoPin = 5;
int relayThreePin = 6;

int alarmHornPin = 7;
int alarmSilencePin = 15;
int alarmTestPin = 16;

int displayEnablePin = 17;
int displayClockPin = 18;
int displayDataPin = 8;

//POTENTIOMETER TESTING
//int voltageInPin = 3;
//int sensorValue = 0;

int ledAlarmPin = 40;

//ANALOG INPUT/OUTPUT PINS
int analogInputPin = 3;
int i2cClockPin = 36;
int i2cDataPin = 37;

//I2C BUS ADDRESSES
int analogOutputI2cAddress = 15;

//ANALOG INPUT VALUE
int analogInputValue = 0;

//ANALOG OUTPUT INITIALIZATION
AD56X2 analogOutput = AD56X2(analogOutputI2cAddress);

bool boardTestMode = false;

bool isRelayOneLowLimit = true;
bool isRelayTwoLowLimit = true;
bool isRelayThreeLowLimit = true;

double relayOneLimit = -1.0;
double relayTwoLimit = -1.0;
double relayThreeLimit = -1.0;

bool isUnitMeters = true;

double emptyLevel = -1.0;
double fullLevel = -1.0;
double tankHeight = -1.0;

bool relayOneAlarmActive = false;
bool relayTwoAlarmActive = false;
bool relayThreeAlarmActive = false;

bool relayOneAlarmTriggered = false;
bool relayTwoAlarmTriggered = false;
bool relayThreeAlarmTriggered = false;

double ma4Level = -1.0;
double ma20Level = -1.0;

/* Modbus stuff */
	//-- moved to arduinoGlue.h // #define MODBUS_RXE_PIN 1 // RxEn pin
	//-- moved to arduinoGlue.h // #define MODBUS_TXE_PIN 2 // TxEn pin
	//-- moved to arduinoGlue.h // #define MODBUS_RX_PIN 44 // Rx pin
	//-- moved to arduinoGlue.h // #define MODBUS_TX_PIN 43 // Tx pin
	//-- moved to arduinoGlue.h // #define MODBUS_SERIAL_BAUD 9600 // Baud rate for esp32 and max485 communication

//Level Sensor Modbus Device ID
int levelSensorModbusID = 1;

//Current Level data register of level sensor
uint16_t level_register = 0x0001;
int currentDepth = 0;
int probeErrorCount = 0;
bool probeErrorOccurred = false;

//Baud Rate data register of level sensor
uint16_t baud_register = 0x0004;
bool baudRateWriteSuccess = false;

//Measure Range data register of level sensor
uint16_t range_register = 0x0002;
bool measureRangeWriteSuccess = false;

//Initialize the ModbusMaster object as node
ModbusMaster node;

const int displayAdjustmentFactor = 10;
const double depthUnitsPerMeter = 1000.0;
const double metersToFeetConversionFactor = 3.28084;
const double feetToMetersConversionFactor = 0.3048;
const double metersToDepthUnitsConversionFactor = 1000.0;
const double feetToDepthUnitsConversionFactor = 304.8;

//POTENTIOMETER TESTING
//const double sensorUnitsPerMeter = 4095.0 / 10.0;
//const int wholeSensorUnitConversionValue = 100;

// Define Modbus parameters
//const byte slaveAddress = 0x01;          // Address of the Modbus slave device
//const byte functionCode = 0x03;          // Function code to read holding registers
//const byte startAddressHigh = 0x00;      // High byte of the starting address
//const byte startAddressLow = 0x02;       // Low byte of the starting address
//const byte registerCountHigh = 0x00;     // High byte of the number of registers to read
//const byte registerCountLow = 0x05;      // Low byte of the number of registers to read

//WAIT INTERVALS
const short levelSensorReadWaitInterval = 5000;
unsigned long long previousLevelSensorReadMillis = 0;
unsigned long long currentLevelSensorReadMillis = 0;
const int bluetoothWaitInterval = 60000;
unsigned long long previousBluetoothMillis = 0;
unsigned long long currentBluetoothMillis = 0;
const short ledAlarmBlinkInterval = 500;
unsigned long long previousLedAlarmBlinkMillis = 0;
unsigned long long currentLedAlarmBlinkMillis = 0;

//INTERNAL STORAGE
Preferences preferences;
String pnamespace = String("InternalInfo");
String isUnitMetersKey = String("UnitM");
String emptyLevelKey = String("ELevel");
String fullLevelKey = String("FLevel");
String tankHeightKey = String("THeight");
String isRelayOneLowLimitKey = String("R1Low");
String isRelayTwoLowLimitKey = String("R2Low");
String isRelayThreeLowLimitKey = String("R3Low");
String relayOneLimitKey = String("R1Lim");
String relayTwoLimitKey = String("R2Lim");
String relayThreeLimitKey = String("R3Lim");
String relayOneAlarmActiveKey = String("A1Act");
String relayTwoAlarmActiveKey = String("A2Act");
String relayThreeAlarmActiveKey = String("A3Act");
String ma4LevelKey = String("4maLevel");
String ma20LevelKey = String("20maLevel");

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

void setLevelSensorBaudRate() {
  //MODBUS WRITING
  uint8_t result;

  // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
  node.setTransmitBuffer(0, lowWord((int)(MODBUS_SERIAL_BAUD)));

  result = node.writeMultipleRegisters(baud_register, 1);
  if (result != node.ku8MBSuccess) {
    Serial.println("Baud Rate Write Failed!");
  } else {
    Serial.println("Baud Rate Write Successful!");
    baudRateWriteSuccess = true;
  }
}

void setLevelSensorMeasureRange() {
  //MODBUS WRITING
  uint8_t result;

  // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
  if (isUnitMeters) {
    node.setTransmitBuffer(0, lowWord((int)(tankHeight * depthUnitsPerMeter)));
  } else {
    node.setTransmitBuffer(0, lowWord((int)(tankHeight * feetToMetersConversionFactor * depthUnitsPerMeter)));
  }

  result = node.writeMultipleRegisters(range_register, 1);
  if (result != node.ku8MBSuccess) {
    Serial.println("Measure Range Write Failed!");
  } else {
    Serial.println("Measure Range Write Successful!");
    measureRangeWriteSuccess = true;
  }
}

int readAnalogInputValue() {
  int readValue = analogRead(analogInputPin);
  return readValue;
}

int readLevelSensorValue() {
  //POTENTIOMETER TESTING
  //int readValue = analogRead(voltageInPin);
  //Serial.println(readValue);
  
  //MODBUS READING
  uint8_t result;
  uint16_t currentLevel = -1;
  uint16_t measureRange = -1;
  int calculatedDepth = -1;

  //Modbus function 0x03 Read Holding Registers according to level sensor datasheet
  result = node.readHoldingRegisters(level_register, 2);
  if (result == node.ku8MBSuccess) {
    Serial.print("Success, Received data: ");
          
    //Retrieve the data from getResponseBuffer(uint8_t u8Index) function.
    //That is, return 16-bit data.
    currentLevel = node.getResponseBuffer(0);
    measureRange = node.getResponseBuffer(1);
    //calculatedDepth = measureRange - currentLevel;
    calculatedDepth = currentLevel;
    Serial.print(currentLevel);
    Serial.print(", ");
    Serial.print(measureRange);
    Serial.println("");
    probeErrorCount = 0;
    probeErrorOccurred = false;
    return calculatedDepth;
  } else {
    Serial.print("Failed, Response Code: ");
    Serial.print(result, HEX);
    Serial.println("");
    probeErrorCount++;
    if (probeErrorCount > 3) {
      probeErrorOccurred = true;
      digitalWrite(relayOnePin, LOW);
      digitalWrite(relayTwoPin, LOW);
      digitalWrite(relayThreePin, LOW);
      digitalWrite(ledAlarmPin, LOW);
      digitalWrite(alarmHornPin, LOW);
      analogOutput.setOutputLevel((uint16_t)(0));
      relayOneAlarmTriggered = false;
      relayTwoAlarmTriggered = false;
      relayThreeAlarmTriggered = false;
      ledAlarmOn = true;
    }
    return currentDepth;
  }
}

void checkRelayOneConditions(double levelSensorValue) {
  if (!isUnitMeters) {
    levelSensorValue *= metersToFeetConversionFactor;
  }

  if (isRelayOneLowLimit) {
    if (levelSensorValue <= relayOneLimit) {
      if (!relayOneAlarmTriggered) {
        digitalWrite(relayOnePin, HIGH);
        if (relayOneAlarmActive) {
          digitalWrite(ledAlarmPin, HIGH);
        }
        if (!silenceOverride && relayOneAlarmActive) {
          digitalWrite(alarmHornPin, HIGH);
        }
        relayOneAlarmTriggered = true;
      }
    }
    else {
      digitalWrite(relayOnePin, LOW);
      if (!(relayTwoAlarmActive && relayTwoAlarmTriggered) && !(relayThreeAlarmActive && relayThreeAlarmTriggered)) {
        digitalWrite(alarmHornPin, LOW);
        digitalWrite(ledAlarmPin, LOW);
        ledAlarmOn = true;
      }
      relayOneAlarmTriggered = false;
    }
  }
  else {
    if (levelSensorValue >= relayOneLimit) {
      if (!relayOneAlarmTriggered) {
        digitalWrite(relayOnePin, HIGH);
        if (relayOneAlarmActive) {
          digitalWrite(ledAlarmPin, HIGH);
        }
        if (!silenceOverride && relayOneAlarmActive) {
          digitalWrite(alarmHornPin, HIGH);
        }
        relayOneAlarmTriggered = true;
      }
    }
    else {
      digitalWrite(relayOnePin, LOW);
      if (!(relayTwoAlarmActive && relayTwoAlarmTriggered) && !(relayThreeAlarmActive && relayThreeAlarmTriggered)) {
        digitalWrite(alarmHornPin, LOW);
        digitalWrite(ledAlarmPin, LOW);
        ledAlarmOn = true;
      }
      relayOneAlarmTriggered = false;
    }
  }
}

void checkRelayTwoConditions(double levelSensorValue) {
  if (!isUnitMeters) {
    levelSensorValue *= metersToFeetConversionFactor;
  }

  if (isRelayTwoLowLimit) {
    if (levelSensorValue <= relayTwoLimit) {
      if (!relayTwoAlarmTriggered) {
        digitalWrite(relayTwoPin, HIGH);
        if (relayTwoAlarmActive) {
          digitalWrite(ledAlarmPin, HIGH);
        }
        if (!silenceOverride && relayTwoAlarmActive) {
          digitalWrite(alarmHornPin, HIGH);
        }
        relayTwoAlarmTriggered = true;
      }
    }
    else {
      digitalWrite(relayTwoPin, LOW);
      if (!(relayOneAlarmActive && relayOneAlarmTriggered) && !(relayThreeAlarmActive && relayThreeAlarmTriggered)) {
        digitalWrite(alarmHornPin, LOW);
        digitalWrite(ledAlarmPin, LOW);
        ledAlarmOn = true;
      }
      relayTwoAlarmTriggered = false;
    }
  }
  else {
    if (levelSensorValue >= relayTwoLimit) {
      if (!relayTwoAlarmTriggered) {
        digitalWrite(relayTwoPin, HIGH);
        if (relayTwoAlarmActive) {
          digitalWrite(ledAlarmPin, HIGH);
        }
        if (!silenceOverride && relayTwoAlarmActive) {
          digitalWrite(alarmHornPin, HIGH);
        }
        relayTwoAlarmTriggered = true;
      }
    }
    else {
      digitalWrite(relayTwoPin, LOW);
      if (!(relayOneAlarmActive && relayOneAlarmTriggered) && !(relayThreeAlarmActive && relayThreeAlarmTriggered)) {
        digitalWrite(alarmHornPin, LOW);
        digitalWrite(ledAlarmPin, LOW);
        ledAlarmOn = true;
      }
      relayTwoAlarmTriggered = false;
    }
  }
}

void checkRelayThreeConditions(double levelSensorValue) {
  if (!isUnitMeters) {
    levelSensorValue *= metersToFeetConversionFactor;
  }

  if (isRelayThreeLowLimit) {
    if (levelSensorValue <= relayThreeLimit) {
      if (!relayThreeAlarmTriggered) {
        digitalWrite(relayThreePin, HIGH);
        if (relayThreeAlarmActive) {
          digitalWrite(ledAlarmPin, HIGH);
        }
        if (!silenceOverride && relayThreeAlarmActive) {
          digitalWrite(alarmHornPin, HIGH);
        }
        relayThreeAlarmTriggered = true;
      }
    }
    else {
      digitalWrite(relayThreePin, LOW);
      if (!(relayOneAlarmActive && relayOneAlarmTriggered) && !(relayTwoAlarmActive && relayTwoAlarmTriggered)) {
        digitalWrite(alarmHornPin, LOW);
        digitalWrite(ledAlarmPin, LOW);
        ledAlarmOn = true;
      }
      relayThreeAlarmTriggered = false;
    }
  }
  else {
    if (levelSensorValue >= relayThreeLimit) {
      if (!relayThreeAlarmTriggered) {
        digitalWrite(relayThreePin, HIGH);
        if (relayThreeAlarmActive) {
          digitalWrite(ledAlarmPin, HIGH);
        }
        if (!silenceOverride && relayThreeAlarmActive) {
          digitalWrite(alarmHornPin, HIGH);
        }
        relayThreeAlarmTriggered = true;
      }
    }
    else {
      digitalWrite(relayThreePin, LOW);
      if (!(relayOneAlarmActive && relayOneAlarmTriggered) && !(relayTwoAlarmActive && relayTwoAlarmTriggered)) {
        digitalWrite(alarmHornPin, LOW);
        digitalWrite(ledAlarmPin, LOW);
        ledAlarmOn = true;
      }
      relayThreeAlarmTriggered = false;
    }
  }
}

void setAnalogOutputValue(double levelSensorValue) {
  if (!isUnitMeters) {
    levelSensorValue *= metersToFeetConversionFactor;
  }

  if (levelSensorValue > ma20Level) {
    levelSensorValue = ma20Level;
  }

  if (levelSensorValue < ma4Level) {
    levelSensorValue = ma4Level;
  }

  double depthUnitsPerMilliamp = (ma20Level - ma4Level) / (16.0);
  double calculatedMilliamps = ((levelSensorValue - ma4Level) / (depthUnitsPerMilliamp)) + (4.0);

  analogOutput.setOutputLevel((uint16_t)((4095.0 / 20.0) * calculatedMilliamps));
}

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    BLEServerCallbacks::onConnect(pServer);
    Serial.println("BLE CONNECTION DETECTED!");
    stopBluetooth();
  };

  void onDisconnect(BLEServer* pServer) {
    BLEServerCallbacks::onDisconnect(pServer);
    Serial.println("BLE CONNECTION LOST!");
    simulationMode = false;
    // Cancel OTA update on disconnect
    if (otaInProgress) {
      Serial.println("[OTA] Update cancelled.");
      Update.end(false); // Abort the update process
      otaInProgress = false;
    }
  }
};

class MyConfigCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    relayOneAlarmTriggered = false;
    relayTwoAlarmTriggered = false;
    relayThreeAlarmTriggered = false;

    String rxValue = String(pCharacteristic->getValue().c_str());
    Serial.println(rxValue);

    if (rxValue.length() > 1) {
      if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("s1")) {
        int isRelayOneLowLimitInt = rxValue.substring(0, rxValue.indexOf('_')).toInt();
        isRelayOneLowLimit = isRelayOneLowLimitInt;
        storeConfigValue(isRelayOneLowLimitKey, String(isRelayOneLowLimitInt));
      }
      else if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("s2")) {
        int isRelayTwoLowLimitInt = rxValue.substring(0, rxValue.indexOf('_')).toInt();
        isRelayTwoLowLimit = isRelayTwoLowLimitInt;
        storeConfigValue(isRelayTwoLowLimitKey, String(isRelayTwoLowLimitInt));
      }
      else if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("s3")) {
        int isRelayThreeLowLimitInt = rxValue.substring(0, rxValue.indexOf('_')).toInt();
        isRelayThreeLowLimit = isRelayThreeLowLimitInt;
        storeConfigValue(isRelayThreeLowLimitKey, String(isRelayThreeLowLimitInt));
      }
      else if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("l1")) {
        relayOneLimit = rxValue.substring(0, rxValue.indexOf('_')).toDouble();
        storeConfigValue(relayOneLimitKey, String(relayOneLimit));
      }
      else if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("l2")) {
        relayTwoLimit = rxValue.substring(0, rxValue.indexOf('_')).toDouble();
        storeConfigValue(relayTwoLimitKey, String(relayTwoLimit));
      }
      else if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("l3")) {
        relayThreeLimit = rxValue.substring(0, rxValue.indexOf('_')).toDouble();
        storeConfigValue(relayThreeLimitKey, String(relayThreeLimit));
      }
      else if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("el")) {
        emptyLevel = rxValue.substring(0, rxValue.indexOf('_')).toDouble();
        storeConfigValue(emptyLevelKey, String(emptyLevel));
      }
      else if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("fl")) {
        fullLevel = rxValue.substring(0, rxValue.indexOf('_')).toDouble();
        storeConfigValue(fullLevelKey, String(fullLevel));
      }
      else if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("th")) {
        tankHeight = rxValue.substring(0, rxValue.indexOf('_')).toDouble();
        storeConfigValue(tankHeightKey, String(tankHeight));
      }
      else if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("un")) {
        int isUnitMetersInt = rxValue.substring(0, rxValue.indexOf('_')).toInt();
        isUnitMeters = isUnitMetersInt;
        storeConfigValue(isUnitMetersKey, String(isUnitMetersInt));
      }
      else if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("a1")) {
        int relayOneAlarmActiveInt = rxValue.substring(0, rxValue.indexOf('_')).toInt();
        relayOneAlarmActive = relayOneAlarmActiveInt;
        storeConfigValue(relayOneAlarmActiveKey, String(relayOneAlarmActiveInt));
      }
      else if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("a2")) {
        int relayTwoAlarmActiveInt = rxValue.substring(0, rxValue.indexOf('_')).toInt();
        relayTwoAlarmActive = relayTwoAlarmActiveInt;
        storeConfigValue(relayTwoAlarmActiveKey, String(relayTwoAlarmActiveInt));
      }
      else if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("a3")) {
        int relayThreeAlarmActiveInt = rxValue.substring(0, rxValue.indexOf('_')).toInt();
        relayThreeAlarmActive = relayThreeAlarmActiveInt;
        storeConfigValue(relayThreeAlarmActiveKey, String(relayThreeAlarmActiveInt));
      }
      else if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("4m")) {
        ma4Level = rxValue.substring(0, rxValue.indexOf('_')).toDouble();
        storeConfigValue(ma4LevelKey, String(ma4Level));
      }
      else if (rxValue.substring(rxValue.length() - 3).equalsIgnoreCase("20m")) {
        ma20Level = rxValue.substring(0, rxValue.indexOf('_')).toDouble();
        storeConfigValue(ma20LevelKey, String(ma20Level));
      }
      else {
        Serial.println("INVALID SETTING SUFFIX!");
      }
    }
    else {
      Serial.println("INVALID MESSAGE!");
    }

    measureRangeWriteSuccess = false;
    pDisplayConfigCharacteristic->setValue(createDisplayConfigString().c_str());
  }
};

class MySimulationCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = String(pCharacteristic->getValue().c_str());
    Serial.println(rxValue);

    if (rxValue.length() > 1) {
      if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("sm")) {
        if (rxValue.substring(0, rxValue.indexOf('_')).equalsIgnoreCase("0")) {
          simulationMode = false;
        } else if (rxValue.substring(0, rxValue.indexOf('_')).equalsIgnoreCase("1")) {
          simulationMode = true;
        } else {
          Serial.println("INVALID SIMULATION MODE VALUE!");
        }
      } else if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("ma")) {
        if (rxValue.substring(0, rxValue.indexOf('_')).equalsIgnoreCase("4")) {
          analogOutput.setOutputLevel((uint16_t)((4095.0 / 20.0) * 4.0));
        } else if (rxValue.substring(0, rxValue.indexOf('_')).equalsIgnoreCase("12")) {
          analogOutput.setOutputLevel((uint16_t)((4095.0 / 20.0) * 12.0));
        } else if (rxValue.substring(0, rxValue.indexOf('_')).equalsIgnoreCase("20")) {
          analogOutput.setOutputLevel((uint16_t)((4095.0 / 20.0) * 20.0));
        } else {
          Serial.println("INVALID mA VALUE!");
        }
      } else if (rxValue.substring(rxValue.length() - 2).equalsIgnoreCase("sv")) {
        if (rxValue.substring(0, rxValue.indexOf('_')).toDouble() <= ma20Level && rxValue.substring(0, rxValue.indexOf('_')).toDouble() >= ma4Level) {
          if (isUnitMeters) {
            currentDepth = (int)(rxValue.substring(0, rxValue.indexOf('_')).toDouble() * metersToDepthUnitsConversionFactor);
          } else {
            currentDepth = (int)(rxValue.substring(0, rxValue.indexOf('_')).toDouble() * feetToDepthUnitsConversionFactor);
          }
          displayIntLong((long)(currentDepth / displayAdjustmentFactor));
          checkRelayOneConditions(currentDepth / depthUnitsPerMeter);
          checkRelayTwoConditions(currentDepth / depthUnitsPerMeter);
          checkRelayThreeConditions(currentDepth / depthUnitsPerMeter);
          setAnalogOutputValue(currentDepth / depthUnitsPerMeter);
        } else {
          Serial.println("INVALID LEVEL VALUE!");
        }
      } else {
        Serial.println("INVALID COMMAND SUFFIX!");
      }
    } else {
      Serial.println("INVALID MESSAGE!");
    }
  }
};

class MyUpdateCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String data = pCharacteristic->getValue().c_str();
    Serial.print("DT: ");
    Serial.println(data);

    if (data.equalsIgnoreCase("0_um")) {
      if (Update.begin(1200000)) {
        otaInProgress = true;
        Serial.println("START UPDATE!");
      }
    } else if (data.equalsIgnoreCase("1_um")) {
      otaInProgress = false;
      fwUpdateFlag = true;
    } else if (otaInProgress) {
      /*
      String byteVals = String("");
      long index = data.indexOf("x");
      long previousIndex = 0;
      long byteCount = 0;
      while (index != -1) {
        byteVals += data.substring(index + 1, index + 3);
        previousIndex = index;
        index = data.indexOf("x", previousIndex + 1);
        byteCount++;
      }
      Serial.println(byteVals);
      */

      long byteCount = data.length() / 2;
      Serial.print("BC: ");
      Serial.println(byteCount);
      byte updateBuffer[byteCount];
      hexCharacterStringToBytes(updateBuffer, data.c_str());

      otaReceived += Update.write(updateBuffer, byteCount);
      Serial.print("WR: ");
      Serial.println(otaReceived);

      fwWriteFlag = true;
    }
  }
};

// Pins 1 and 2 made high for Modbus transmision mode
void modbusPreTransmission() {
  delay(20);
  digitalWrite(MODBUS_RXE_PIN, HIGH);
  digitalWrite(MODBUS_TXE_PIN, HIGH);
}

// Pins 1 and 2 made low for Modbus receive mode
void modbusPostTransmission() {
  digitalWrite(MODBUS_RXE_PIN, LOW);
  digitalWrite(MODBUS_TXE_PIN, LOW);
  delay(20);
}

void storeConfigValue(String key, String value) {
  preferences.begin(pnamespace.c_str());
  preferences.putString(key.c_str(), value.c_str());
  preferences.end();
}

String findConfigValue(String key, String defaultValue) {
  String configValue = String("-1");
  preferences.begin(pnamespace.c_str());
  configValue = preferences.getString(key.c_str(), defaultValue.c_str());
  preferences.end();
  return configValue;
}

void debounceBluetoothResetButton() {
  if (!digitalRead(bluetoothResetPin)) {
    bluetoothResetButtonPressed = true;
  }
  if (digitalRead(bluetoothResetPin) && bluetoothResetButtonPressed) {
    bluetoothResetButtonPressed = false;
    resetBluetooth();
  }
}

void debounceAlarmTestButton() {
  if (digitalRead(alarmTestPin) && !alarmTestButtonPressed) {
    alarmTestButtonPressed = true;
    testOverride = true;
    boardTestSequence();
  }
  if (!digitalRead(alarmTestPin) && alarmTestButtonPressed) {
    alarmTestButtonPressed = false;
    testOverride = false;
    resetBoard();
  }
}

void debounceAlarmSilenceButton() {
  if (digitalRead(alarmSilencePin)) {
    alarmSilenceButtonPressed = true;
    silenceOverride = true;
    digitalWrite(alarmHornPin, LOW);
  }
  if (!digitalRead(alarmSilencePin) && alarmSilenceButtonPressed) {
    alarmSilenceButtonPressed = false;
    silenceOverride = false;
  }
}

void boardTestSequence() {
  //LEVELMINDER BOARD TESTING SEQUENCE
  boardTestMode = true;

  digitalWrite(relayOnePin, HIGH);
  digitalWrite(relayTwoPin, HIGH);
  digitalWrite(relayThreePin, HIGH);

  digitalWrite(ledAlarmPin, HIGH);

  digitalWrite(alarmHornPin, HIGH);

  analogOutput.setOutputLevel((uint16_t)(4095));

  displayIntLong(8888);

  //relayOneAlarmTriggered = true;
  //relayTwoAlarmTriggered = true;
  //relayThreeAlarmTriggered = true;
}

void resetBoard() {
  //LEVELMINDER BOARD TESTING SEQUENCE
  digitalWrite(relayOnePin, LOW);
  digitalWrite(relayTwoPin, LOW);
  digitalWrite(relayThreePin, LOW);

  digitalWrite(ledAlarmPin, LOW);

  digitalWrite(alarmHornPin, LOW);

  analogOutput.setOutputLevel((uint16_t)(0));

  displayIntLong(0);

  boardTestMode = false;
  ledAlarmOn = true;

  //relayOneAlarmTriggered = false;
  //relayTwoAlarmTriggered = false;
  //relayThreeAlarmTriggered = false;
}

void resetBluetooth() {
  BLEDevice::startAdvertising();
  bluetoothActive = true;
  previousBluetoothMillis = millis();
}

void stopBluetooth() {
  bluetoothActive = false;
  BLEDevice::stopAdvertising();
}

String createDisplayConfigString() {
  return String((int)(isUnitMeters)) + "_un&" + String(emptyLevel) + "_el&" + String(fullLevel) + "_fl&" + String(tankHeight) + "_th&" +
         String(relayOneLimit) + "_l1&" + String(relayTwoLimit) + "_l2&" + String(relayThreeLimit) + "_l3&" +
         String((int)(isRelayOneLowLimit)) + "_s1&" + String((int)(isRelayTwoLowLimit)) + "_s2&" + String((int)(isRelayThreeLowLimit)) + "_s3&" +
         String((int)(relayOneAlarmActive)) + "_a1&" + String((int)(relayTwoAlarmActive)) + "_a2&" + String((int)(relayThreeAlarmActive)) + "_a3&" +
         String(ma4Level) + "_4m&" + String(ma20Level) + "_20m&";
}

void hexCharacterStringToBytes(byte *byteArray, const char *hexString) {
  bool oddLength = strlen(hexString) & 1;

  byte currentByte = 0;
  int byteIndex = 0;

  for (int charIndex = 0; charIndex < strlen(hexString); charIndex++) {
    bool oddCharIndex = charIndex & 1;

    if (oddLength) {
      // If the length is odd
      if (oddCharIndex) {
        // odd characters go in high nibble
        currentByte = nibble(hexString[charIndex]) << 4;
      } else {
        // Even characters go into low nibble
        currentByte |= nibble(hexString[charIndex]);
        byteArray[byteIndex++] = currentByte;
        currentByte = 0;
      }
    } else {
      // If the length is even
      if (!oddCharIndex) {
        // Odd characters go into the high nibble
        currentByte = nibble(hexString[charIndex]) << 4;
      }
      else {
        // Odd characters go into low nibble
        currentByte |= nibble(hexString[charIndex]);
        byteArray[byteIndex++] = currentByte;
        currentByte = 0;
      }
    }
  }
}

byte nibble(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';

  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;

  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;

  return 0;  // Not a valid hexadecimal character
}

void rebootEspWithReason(String reason) {
  Serial.println(reason);
  delay(1000);
  ESP.restart();
}

void setup() {
  pinMode(bluetoothResetPin, INPUT_PULLUP);

  pinMode(relayOnePin, OUTPUT);
  pinMode(relayTwoPin, OUTPUT);
  pinMode(relayThreePin, OUTPUT);
  
  pinMode(alarmHornPin, OUTPUT);
  pinMode(alarmSilencePin, INPUT);
  pinMode(alarmTestPin, INPUT);

  pinMode(displayEnablePin, OUTPUT);
  pinMode(displayClockPin, OUTPUT);
  pinMode(displayDataPin, OUTPUT);
  
  pinMode(ledAlarmPin, OUTPUT);

  pinMode(MODBUS_RXE_PIN, OUTPUT);
  pinMode(MODBUS_TXE_PIN, OUTPUT);
  digitalWrite(MODBUS_RXE_PIN, LOW);
  digitalWrite(MODBUS_TXE_PIN, LOW);

  isUnitMeters = findConfigValue(isUnitMetersKey).toInt();

  emptyLevel = findConfigValue(emptyLevelKey).toDouble();
  fullLevel = findConfigValue(fullLevelKey, String("30.0")).toDouble();
  tankHeight = findConfigValue(tankHeightKey, String("30.0")).toDouble();

  isRelayOneLowLimit = findConfigValue(isRelayOneLowLimitKey, String("1")).toInt();
  isRelayTwoLowLimit = findConfigValue(isRelayTwoLowLimitKey, String("1")).toInt();
  isRelayThreeLowLimit = findConfigValue(isRelayThreeLowLimitKey, String("1")).toInt();

  relayOneLimit = findConfigValue(relayOneLimitKey).toDouble();
  relayTwoLimit = findConfigValue(relayTwoLimitKey).toDouble();
  relayThreeLimit = findConfigValue(relayThreeLimitKey).toDouble();

  relayOneAlarmActive = findConfigValue(relayOneAlarmActiveKey).toInt();
  relayTwoAlarmActive = findConfigValue(relayTwoAlarmActiveKey).toInt();
  relayThreeAlarmActive = findConfigValue(relayThreeAlarmActiveKey).toInt();

  ma4Level = findConfigValue(ma4LevelKey).toDouble();
  ma20Level = findConfigValue(ma20LevelKey, String("30.0")).toDouble();

  displayOn();

  Serial.begin(MODBUS_SERIAL_BAUD);
  //while (!Serial) {
  //}

  //Serial2.begin(baud-rate, protocol, RX pin, TX pin);.
  Serial2.begin(MODBUS_SERIAL_BAUD, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  Serial2.setTimeout(200);

  //modbus slave ID 1
  node.begin(levelSensorModbusID, Serial2);

  //  callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(modbusPreTransmission);
  node.postTransmission(modbusPostTransmission);

  Wire.begin(i2cDataPin, i2cClockPin);
  //BR = 12;  // 400 kHz (maximum)

  analogOutput.begin();

  BLEDevice::init("RLM-3C");
  BLEDevice::setMTU(BLE_MTU);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pSendConfigService = pServer->createService(SEND_CONFIG_SERVICE_UUID);
  BLEService *pDisplayConfigService = pServer->createService(DISPLAY_CONFIG_SERVICE_UUID);
  BLEService *pSendSimulationService = pServer->createService(SEND_SIMULATION_SERVICE_UUID);
  BLEService *pSendUpdateService = pServer->createService(SEND_UPDATE_SERVICE_UUID);
  BLECharacteristic *pSendConfigCharacteristic = pSendConfigService->createCharacteristic(
                                         SEND_CONFIG_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  pDisplayConfigCharacteristic = pDisplayConfigService->createCharacteristic(
                                         DISPLAY_CONFIG_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  BLECharacteristic *pSendSimulationCharacteristic = pSendSimulationService->createCharacteristic(
                                         SEND_SIMULATION_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  BLECharacteristic *pSendUpdateCharacteristic = pSendUpdateService->createCharacteristic(
                                         SEND_UPDATE_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  pUpdateProgressCharacteristic = pSendUpdateService->createCharacteristic(
                                         UPDATE_PROGRESS_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  pSendConfigCharacteristic->setCallbacks(new MyConfigCharacteristicCallbacks());
  pSendSimulationCharacteristic->setCallbacks(new MySimulationCharacteristicCallbacks());
  pSendUpdateCharacteristic->setCallbacks(new MyUpdateCharacteristicCallbacks());

  pSendConfigCharacteristic->setValue(FIRMWARE_VERSION);
  pDisplayConfigCharacteristic->setValue(createDisplayConfigString().c_str());
  pSendSimulationCharacteristic->setValue("SEND LEVELMINDER SIMULATION VALUES HERE!");
  pSendUpdateCharacteristic->setValue("SEND LEVELMINDER UPDATE FILE HERE!");
  pUpdateProgressCharacteristic->setValue("0_upProg");
  pSendConfigService->start();
  pDisplayConfigService->start();
  pSendSimulationService->start();
  pSendUpdateService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SEND_CONFIG_SERVICE_UUID);
  pAdvertising->addServiceUUID(DISPLAY_CONFIG_SERVICE_UUID);
  pAdvertising->addServiceUUID(SEND_SIMULATION_SERVICE_UUID);
  pAdvertising->addServiceUUID(SEND_UPDATE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  //pAdvertising->setMinPreferred(0x00);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connection issues
  pAdvertising->setMinPreferred(0x12);  // -^
  //BLEDevice::startAdvertising();

  // Initialize Outputs
  digitalWrite(relayOnePin, LOW);
  digitalWrite(relayTwoPin, LOW);
  digitalWrite(relayThreePin, LOW);

  digitalWrite(ledAlarmPin, LOW);

  digitalWrite(alarmHornPin, LOW);

  analogOutput.setOutputLevel((uint16_t)(0));

  // Initialize Level Sensor Baud Rate
  setLevelSensorBaudRate();

  // Allow some time for initialization
  delay(2000);

  // Initialize Level Sensor Measure Range
  setLevelSensorMeasureRange();
}

void loop() {
  //CHECK FOR AVAILABLE FIRMWARE UPDATE
  if (fwWriteFlag) {
    fwWriteFlag = false;
    String updateProgressString = String(otaReceived);
    updateProgressString += "_upProg";
    pUpdateProgressCharacteristic->setValue(updateProgressString.c_str());
    pUpdateProgressCharacteristic->notify();
    Serial.print("NT: ");
    Serial.println(updateProgressString);
  }

  if (fwUpdateFlag) {
    fwUpdateFlag = false;
    Serial.print("WR: ");
    Serial.println(otaReceived);
    if (!Update.end(true)) {
      Serial.print("UPDATE ERROR: ");
      Serial.println(Update.getError());
    } else {
      Serial.println("END UPDATE!");
    }
    if (Update.isFinished()) {
      rebootEspWithReason("FIRMWARE UPDATE!");
    }
  }

  //CHECK LEVEL SENSOR BAUD RATE
  if (!baudRateWriteSuccess) {
    setLevelSensorBaudRate();
  }

  //CHECK LEVEL SENSOR MEASURE RANGE
  if (!measureRangeWriteSuccess) {
    setLevelSensorMeasureRange();
  }

  //DEBOUNCE BUTTONS
  debounceBluetoothResetButton();
  debounceAlarmTestButton();
  debounceAlarmSilenceButton();

  //Serial.println("BT: " + String(digitalRead(bluetoothResetPin)));
  //Serial.println("Test: " + String(digitalRead(alarmTestPin)));
  //Serial.println("Silence: " + String(digitalRead(alarmSilencePin)));
  //Serial.println("UpInProgress: " + String(otaInProgress));

  //POTENTIOMETER TESTING
  //if (!(testOverride || simulationMode)) {
    //sensorValue = readLevelSensorValue();
    //displayIntLong((long)((sensorValue / sensorUnitsPerMeter) * wholeSensorUnitConversionValue));

    //checkRelayOneConditions(sensorValue / sensorUnitsPerMeter);
    //checkRelayTwoConditions(sensorValue / sensorUnitsPerMeter);
    //checkRelayThreeConditions(sensorValue / sensorUnitsPerMeter);
    //setAnalogOutputValue(sensorValue / sensorUnitsPerMeter);
  //}

  //ANALOG INPUT READING
  //if (!(testOverride || simulationMode || otaInProgress)) {
    //Serial.println(String("ANALOG INPUT: ") + String(readAnalogInputValue()));
  //}

  //MODBUS READING
  currentLevelSensorReadMillis = millis();
  if ((!(testOverride || simulationMode || otaInProgress)) && (currentLevelSensorReadMillis - previousLevelSensorReadMillis >= levelSensorReadWaitInterval)) {
    previousLevelSensorReadMillis = currentLevelSensorReadMillis;
    currentDepth = readLevelSensorValue();
    if (!probeErrorOccurred) {
      displayIntLong((long)(currentDepth / displayAdjustmentFactor));
      checkRelayOneConditions(currentDepth / depthUnitsPerMeter);
      checkRelayTwoConditions(currentDepth / depthUnitsPerMeter);
      checkRelayThreeConditions(currentDepth / depthUnitsPerMeter);
      setAnalogOutputValue(currentDepth / depthUnitsPerMeter);
    } else {
      displayProbeMissingError();
    }
  } else {
    delay(50);
  }

  //LED ALARM FLASH
  currentLedAlarmBlinkMillis = millis();
  if (((boardTestMode) || (relayOneAlarmTriggered && relayOneAlarmActive) || (relayTwoAlarmTriggered && relayTwoAlarmActive) || (relayThreeAlarmTriggered && relayThreeAlarmActive)) && (currentLedAlarmBlinkMillis - previousLedAlarmBlinkMillis >= ledAlarmBlinkInterval)) {
    previousLedAlarmBlinkMillis = currentLedAlarmBlinkMillis;
    digitalWrite(ledAlarmPin, ledAlarmOn);
    ledAlarmOn = !ledAlarmOn;
  }

  //DEACTIVATE BLUETOOTH AFTER INACTIVITY
  currentBluetoothMillis = millis();
  if ((bluetoothActive) && (currentBluetoothMillis - previousBluetoothMillis >= bluetoothWaitInterval)) {
    stopBluetooth();
  }
}