#include "LevelSensorV1.h"

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

void boardTestSequence() {
  //LEVELMINDER BOARD TESTING SEQUENCE
  boardTestMode = true;

  digitalWrite(relayOnePin, HIGH);
  digitalWrite(relayTwoPin, HIGH);
  digitalWrite(relayThreePin, HIGH);

  //digitalWrite(ledAlarmPin, HIGH);
  alarmFlashOn();

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

  //digitalWrite(ledAlarmPin, LOW);
  alarmFlashOff();

  digitalWrite(alarmHornPin, LOW);

  analogOutput.setOutputLevel((uint16_t)(0));

  displayIntLong(0);

  boardTestMode = false;
  ledAlarmOn = true;

  relayOneAlarmTriggered = false;
  relayTwoAlarmTriggered = false;
  relayThreeAlarmTriggered = false;
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

double calculateTankVolumeGallons(double diameter, double height) {
  // Calculate the volume of a cylinder in gallons
  // Volume = π * (radius^2) * height
  double radius = diameter / 2.0;
  double volumeCubicLength = PI * (radius * radius) * height; // Volume in cubic feet or cubic meters depending on the units of diameter and height
  double volumeGallons;

  if (isUnitMeters) {
    volumeGallons = volumeCubicLength / 0.0037854118; // Convert cubic meters to gallons (1 gallon = 0.0037854118 cubic meters)
  } else {
    volumeGallons = volumeCubicLength / 0.1336805556; // Convert cubic feet to gallons (1 gallon = 0.1336805556 cubic feet)
  }

  if (isGallonsDisplayX1) {
    volumeGallons = volumeGallons; // Display as is
  } else {
    volumeGallons = volumeGallons / 1000.00; // Divide by 1000.00 for kGals
  }

  return volumeGallons;
}

double calculateTankDepthFromGallons(double diameter, double volume) {
  // Calculate Tank Depth from Volume in Gallons
  double radius = diameter / 2.0;
  double tankDepth;

  if (isGallonsDisplayX1) {
    volume = volume;
  } else {
    volume = volume * 1000.00;
  }

  if (isUnitMeters) {
    tankDepth = volume * 0.0037854118; // Convert gallons to cubic meters (1 gallon = 0.0037854118 cubic meters)
  } else {
    tankDepth = volume * 0.1336805556; // Convert gallons to cubic feet (1 gallon = 0.1336805556 cubic feet)
  }

  return (tankDepth) / (PI * radius * radius);
}