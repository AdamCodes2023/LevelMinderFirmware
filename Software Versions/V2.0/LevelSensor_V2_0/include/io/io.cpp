#include "LevelSensorV1.h"

#define SAMPLES 64 // Multi-sampling to smooth out fluctuating noise

void checkRelayOneConditions(double levelSensorValue) {
  if (!isUnitMeters) {
    levelSensorValue *= metersToFeetConversionFactor;
  }

  if (isRelayOneLowLimit) {
    if (levelSensorValue <= relayOneLimit) {
      if (!relayOneAlarmTriggered) {
        digitalWrite(relayOnePin, HIGH);
        if (relayOneAlarmActive) {
          //digitalWrite(ledAlarmPin, HIGH);
          alarmFlashOn(0); // 0 = Solid Red
        }
        if (!silenceOverride && relayOneAlarmActive && relayOneAlarmAudible) {
          digitalWrite(alarmHornPin, HIGH);
        }
        relayOneAlarmTriggered = true;
      }
    }
    else {
      digitalWrite(relayOnePin, LOW);
      if (!(relayTwoAlarmActive && relayTwoAlarmTriggered && relayTwoAlarmAudible)/* && !(relayThreeAlarmActive && relayThreeAlarmTriggered)*/) {
        digitalWrite(alarmHornPin, LOW);
        //digitalWrite(ledAlarmPin, LOW);
        alarmFlashOff();
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
          //digitalWrite(ledAlarmPin, HIGH);
          alarmFlashOn(0); // 0 = Solid Red
        }
        if (!silenceOverride && relayOneAlarmActive && relayOneAlarmAudible) {
          digitalWrite(alarmHornPin, HIGH);
        }
        relayOneAlarmTriggered = true;
      }
    }
    else {
      digitalWrite(relayOnePin, LOW);
      if (!(relayTwoAlarmActive && relayTwoAlarmTriggered && relayTwoAlarmAudible)/* && !(relayThreeAlarmActive && relayThreeAlarmTriggered)*/) {
        digitalWrite(alarmHornPin, LOW);
        //digitalWrite(ledAlarmPin, LOW);
        alarmFlashOff();
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
          //digitalWrite(ledAlarmPin, HIGH);
          alarmFlashOn(2); // 2 = Solid Yellow
        }
        if (!silenceOverride && relayTwoAlarmActive && relayTwoAlarmAudible) {
          digitalWrite(alarmHornPin, HIGH);
        }
        relayTwoAlarmTriggered = true;
      }
    }
    else {
      digitalWrite(relayTwoPin, LOW);
      if (!(relayOneAlarmActive && relayOneAlarmTriggered && relayOneAlarmAudible)/* && !(relayThreeAlarmActive && relayThreeAlarmTriggered)*/) {
        digitalWrite(alarmHornPin, LOW);
        //digitalWrite(ledAlarmPin, LOW);
        alarmFlashOff();
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
          //digitalWrite(ledAlarmPin, HIGH);
          alarmFlashOn(2); // 2 = Solid Yellow
        }
        if (!silenceOverride && relayTwoAlarmActive && relayTwoAlarmAudible) {
          digitalWrite(alarmHornPin, HIGH);
        }
        relayTwoAlarmTriggered = true;
      }
    }
    else {
      digitalWrite(relayTwoPin, LOW);
      if (!(relayOneAlarmActive && relayOneAlarmTriggered && relayOneAlarmAudible)/* && !(relayThreeAlarmActive && relayThreeAlarmTriggered)*/) {
        digitalWrite(alarmHornPin, LOW);
        //digitalWrite(ledAlarmPin, LOW);
        alarmFlashOff();
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

  if (probeErrorOccurred && relayThreeAlarmActive) {
    digitalWrite(relayThreePin, HIGH);
    /*
    if (relayThreeAlarmActive) {
      //digitalWrite(ledAlarmPin, HIGH);
      alarmFlashOn();
    }
    if (!silenceOverride && relayThreeAlarmActive) {
      digitalWrite(alarmHornPin, HIGH);
    }
    */
    relayThreeAlarmTriggered = true;
  }
  else if (isRelayThreeLowLimit) {
    if (levelSensorValue <= relayThreeLimit) {
      if (!relayThreeAlarmTriggered) {
        digitalWrite(relayThreePin, HIGH);
        /*
        if (relayThreeAlarmActive) {
          //digitalWrite(ledAlarmPin, HIGH);
          alarmFlashOn();
        }
        if (!silenceOverride && relayThreeAlarmActive) {
          digitalWrite(alarmHornPin, HIGH);
        }
        */
        relayThreeAlarmTriggered = true;
      }
    }
    else {
      digitalWrite(relayThreePin, LOW);
      /*
      if (!(relayOneAlarmActive && relayOneAlarmTriggered) && !(relayTwoAlarmActive && relayTwoAlarmTriggered)) {
        digitalWrite(alarmHornPin, LOW);
        //digitalWrite(ledAlarmPin, LOW);
        alarmFlashOff();
        ledAlarmOn = true;
      }
      */
      relayThreeAlarmTriggered = false;
    }
  }
  else {
    if (levelSensorValue >= relayThreeLimit) {
      if (!relayThreeAlarmTriggered) {
        digitalWrite(relayThreePin, HIGH);
        /*
        if (relayThreeAlarmActive) {
          //digitalWrite(ledAlarmPin, HIGH);
          alarmFlashOn();
        }
        if (!silenceOverride && relayThreeAlarmActive) {
          digitalWrite(alarmHornPin, HIGH);
        }
        */
        relayThreeAlarmTriggered = true;
      }
    }
    else {
      digitalWrite(relayThreePin, LOW);
      /*
      if (!(relayOneAlarmActive && relayOneAlarmTriggered) && !(relayTwoAlarmActive && relayTwoAlarmTriggered)) {
        digitalWrite(alarmHornPin, LOW);
        //digitalWrite(ledAlarmPin, LOW);
        alarmFlashOff();
        ledAlarmOn = true;
      }
      */
      relayThreeAlarmTriggered = false;
    }
  }
}

int readAnalogInputValue() {
  int readValue = analogRead(analogInputPin);

  // Debugging: Print the raw analog input value to the serial monitor
  Serial.print("Raw Analog Input Value: ");
  Serial.println(readValue);
  
  return readValue;
}

int convertAnalogInputToDepthUnits(int rawAnalogInput) {
  double milliamps = (double)(rawAnalogInput / rawUnitsPerMilliamp);
  double depth = ((milliamps - 4.0) * ((fullLevel - emptyLevel) / (16.0))) + emptyLevel;

  if (depth < emptyLevel) {
    depth = emptyLevel;
  }

  if (depth > fullLevel) {
    depth = fullLevel;
  }

  if (isUnitMeters) {
    return (int)(depth * metersToDepthUnitsConversionFactor);
  }
  else {
    return (int)(depth * feetToDepthUnitsConversionFactor);
  }
}

int readLevelSensorValue420mA() {
  int rawAnalogInput = readAnalogInputValue();
  if (rawAnalogInput > (int)(rawUnitsPerMilliamp * 3.5)) {
    Serial.print("Success, Received data: ");

    int depthUnits = convertAnalogInputToDepthUnits(rawAnalogInput);

    if (isUnitMeters) {
      Serial.print(depthUnits / metersToDepthUnitsConversionFactor);
    }
    else {
      Serial.print(depthUnits / feetToDepthUnitsConversionFactor);
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

    return depthUnits;
  } else {
    Serial.print("Failed, Raw AI Reading: ");
    Serial.print(rawAnalogInput);
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