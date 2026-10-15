#include "LevelSensorV1.h"

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

// Create a lightweight yielding function
void modbusIdleCPUSafe() {
  // This temporarily releases the CPU core, allowing FreeRTOS 
  // to feed the watchdog timer while waiting for the Level Sensor.
  delay(1);
}

// Function to split a uint32_t into two uint16_t registers in CDAB format
void convertUint32ToModbusCDAB(uint32_t sourceValue, uint16_t* destRegisters) {
    // In CDAB format (Word Swapped):
    // The lower 16 bits (CD) go into the first register
    destRegisters[0] = sourceValue & 0xFFFF;
    
    // The upper 16 bits (AB) go into the second register
    destRegisters[1] = (sourceValue >> 16) & 0xFFFF;
}

void convertDoubleToModbusCDAB(double sourceValue, uint16_t* destRegisters) {
    // 1. Downcast the 64-bit double to a 32-bit single-precision float
    float floatValue = static_cast<float>(sourceValue);

    // 2. Copy the float bits safely into a 32-bit integer for bitwise manipulation
    uint32_t rawBits;
    memcpy(&rawBits, &floatValue, sizeof(rawBits));

    // 3. Extract the 4 bytes (assuming standard Little-Endian CPU architecture like ESP32/ARM/AVR)
    // In ABCD (Big-Endian/Network Order) layout:
    uint8_t a = (rawBits >> 24) & 0xFF;
    uint8_t b = (rawBits >> 16) & 0xFF;
    uint8_t c = (rawBits >> 8)  & 0xFF;
    uint8_t d = (rawBits >> 0)  & 0xFF;

    // 4. Arrange into CDAB format (Word Swapped):
    // Register 0 gets CD, Register 1 gets AB
    destRegisters[0] = (c << 8) | d;
    destRegisters[1] = (a << 8) | b;
}

void setLevelSensorBaudRate() {
  //MODBUS WRITING
  uint8_t result;
  uint16_t modbusRegisters[2];

  convertUint32ToModbusCDAB((uint32_t)(MODBUS_SERIAL_BAUD), modbusRegisters);

  // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
  node.setTransmitBuffer(0, modbusRegisters[0]);
  node.setTransmitBuffer(1, modbusRegisters[1]);

  result = node.writeMultipleRegisters(baud_register, 2);
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
  uint16_t modbusRegisters[2];

  // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
  if (isUnitMeters) {
    convertDoubleToModbusCDAB((double)(tankHeight/* * depthUnitsPerMeter*/), modbusRegisters);
    node.setTransmitBuffer(0, modbusRegisters[0]);
    node.setTransmitBuffer(1, modbusRegisters[1]);
  } else {
    convertDoubleToModbusCDAB((double)(tankHeight * feetToMetersConversionFactor/* * depthUnitsPerMeter*/), modbusRegisters);
    node.setTransmitBuffer(0, modbusRegisters[0]);
    node.setTransmitBuffer(1, modbusRegisters[1]);
  }

  result = node.writeMultipleRegisters(range_register, 2);
  if (result != node.ku8MBSuccess) {
    Serial.println("Measure Range Write Failed!");
  } else {
    Serial.println("Measure Range Write Successful!");
    measureRangeWriteSuccess = true;
  }
}

void setLevelSensorRateSpeed() {
  //MODBUS WRITING
  uint8_t result;

  // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
  node.setTransmitBuffer(0, (uint16_t)(LEVEL_SENSOR_FAST_RATE_SPEED));

  result = node.writeMultipleRegisters(rate_register, 1);
  if (result != node.ku8MBSuccess) {
    Serial.println("Rate Speed Write Failed!");
  } else {
    Serial.println("Rate Speed Write Successful!");
    rateSpeedWriteSuccess = true;
  }
}

float convertLevelReadingToFloat(uint16_t reg1, uint16_t reg2) {
  // Combine the two 16-bit registers into a single 32-bit value
  uint32_t combined = ((uint32_t)reg1 << 16) | reg2;

  // Convert the combined value to a float
  float result;
  memcpy(&result, &combined, sizeof(result));

  return result;
}

int readLevelSensorValueRS485() {
  //POTENTIOMETER TESTING
  //int readValue = analogRead(voltageInPin);
  //Serial.println(readValue);
  
  //MODBUS READING
  uint8_t result;
  float currentLevel = -1;
  //uint16_t measureRange = -1;
  int calculatedDepth = -1;

  //Modbus function 0x04 Read Input Registers according to level sensor datasheet
  result = node.readInputRegisters(level_register, 2);
  if (result == node.ku8MBSuccess) {
    Serial.print("Success, Received data: ");
          
    //Retrieve the data from getResponseBuffer(uint8_t u8Index) function.
    //That is, return 16-bit data.
    //currentLevel = node.getResponseBuffer(0);
    currentLevel = convertLevelReadingToFloat(node.getResponseBuffer(1), node.getResponseBuffer(0));
    //measureRange = node.getResponseBuffer(1);
    //calculatedDepth = measureRange - currentLevel;
    calculatedDepth = currentLevel * depthUnitsPerMeter;

    if (isUnitMeters) {
      Serial.print(currentLevel);
    }
    else {
      Serial.print(currentLevel * metersToFeetConversionFactor);
    }
    Serial.print(", ");
    //Serial.print(measureRange);
    if (isUnitMeters) {
      Serial.print(tankHeight/* * depthUnitsPerMeter*/);
      Serial.print(" meters");
    } else {
      Serial.print(tankHeight/* * feetToMetersConversionFactor * depthUnitsPerMeter*/);
      Serial.print(" feet");
    }
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
      if (!probeErrorOccurred) {
        //digitalWrite(ledAlarmPin, LOW);
        alarmFlashOff();
        digitalWrite(alarmHornPin, LOW);
      }

      probeErrorOccurred = true;
      digitalWrite(relayOnePin, LOW);
      digitalWrite(relayTwoPin, LOW);
      if (relayThreeAlarmActive) {
        digitalWrite(relayThreePin, HIGH);
      } else {
        digitalWrite(relayThreePin, LOW);
      }
      
      analogOutput.setOutputLevel((uint16_t)(0));
      relayOneAlarmTriggered = false;
      relayTwoAlarmTriggered = false;
      relayThreeAlarmTriggered = false;
      ledAlarmOn = true;
    }
    return currentDepth;
  }
}