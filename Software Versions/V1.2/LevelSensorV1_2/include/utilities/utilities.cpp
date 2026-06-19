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