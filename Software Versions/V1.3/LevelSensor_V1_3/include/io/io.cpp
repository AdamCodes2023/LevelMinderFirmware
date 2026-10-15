#include "LevelSensorV1.h"

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

int readAnalogInputValue() {
  int readValue = analogRead(analogInputPin);
  return readValue;
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