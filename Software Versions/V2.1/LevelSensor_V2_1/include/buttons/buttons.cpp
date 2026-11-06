#include "LevelSensorV1.h"

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
  static unsigned long long previousButtonMillis = 0;
  static unsigned long long currentButtonMillis = 0;
  static const unsigned short alarmTestHoldInterval = 3000; // Hold Silence Button for 3 seconds to trigger Test Mode.
  static const unsigned short alarmTestSilenceHoldInterval = 6000; // Hold Silence Button for 6 seconds to silence the Alarm in Test Mode.
  static bool alarmTestSilenceActive = false;

  if (digitalRead(alarmSilencePin)) {
    currentButtonMillis = millis();
    if (!alarmSilenceButtonPressed) {
      previousButtonMillis = currentButtonMillis;
      alarmSilenceButtonPressed = true;
      silenceOverride = true;
      digitalWrite(alarmHornPin, LOW);
    }

    if (currentButtonMillis - previousButtonMillis >= alarmTestHoldInterval && !testOverride) {
      silenceOverride = false;
      testOverride = true;
      boardTestSequence();
    }

    if (currentButtonMillis - previousButtonMillis >= alarmTestSilenceHoldInterval && !alarmTestSilenceActive) {
      alarmTestSilenceActive = true;
      silenceOverride = true;
      digitalWrite(alarmHornPin, LOW);
    }
  }
  if (!digitalRead(alarmSilencePin) && alarmSilenceButtonPressed) {
    if (testOverride) {
      testOverride = false;
      resetBoard();
    }
    alarmSilenceButtonPressed = false;
    alarmTestSilenceActive = false;
    silenceOverride = false;
  }
}