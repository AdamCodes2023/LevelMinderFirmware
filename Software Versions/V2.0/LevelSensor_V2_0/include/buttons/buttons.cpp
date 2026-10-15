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