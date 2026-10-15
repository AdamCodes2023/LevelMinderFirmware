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