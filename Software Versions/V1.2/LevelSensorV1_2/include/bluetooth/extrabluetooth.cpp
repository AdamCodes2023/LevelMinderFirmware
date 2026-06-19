#include "LevelSensorV1.h"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    BLEServerCallbacks::onConnect(pServer);
    Serial.println("BLE CONNECTION DETECTED!");
    bleConnected = true;
    previousBluetoothMillis = currentBluetoothMillis;
    bluetoothVerifyPreviousMillis = bluetoothVerifyCurrentMillis;
    stopBluetooth();
  };

  void onDisconnect(BLEServer* pServer) {
    BLEServerCallbacks::onDisconnect(pServer);
    Serial.println("BLE CONNECTION LOST!");
    bleConnected = false;
    simulationMode = false;
    // Cancel OTA update on disconnect
    if (otaInProgress) {
      Serial.println("[OTA] Update cancelled.");
      Update.end(false); // Abort the update process
      otaInProgress = false;
    }
    pSendConfigCharacteristic->setValue(FIRMWARE_VERSION);
    if (bluetoothVerifyCurrentMillis - bluetoothVerifyPreviousMillis < bluetoothVerifyInterval) {
      resetBluetooth();
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

void forceBLEDisconnect() {
  if (pServer->getConnectedCount() >= 1) {
    pServer->disconnect(pServer->getConnId());
  }
}

void resetBluetooth() {
  // CRITICAL STEP: Set advertisement type to connectable, undirected (ADV_TYPE_IND)
  BLEDevice::getAdvertising()->setAdvertisementType(ADV_TYPE_IND);
  BLEDevice::startAdvertising();
  bluetoothActive = true;
  previousBluetoothMillis = currentBluetoothMillis;
}

void stopBluetooth() {
  bluetoothActive = false;
  BLEDevice::stopAdvertising();
  // CRITICAL STEP: Set advertisement type to non-connectable, undirected (ADV_TYPE_NONCONN_IND)
  BLEDevice::getAdvertising()->setAdvertisementType(ADV_TYPE_NONCONN_IND);
}

String createDisplayConfigString() {
  return String((int)(isUnitMeters)) + "_un&" + String(emptyLevel) + "_el&" + String(fullLevel) + "_fl&" + String(tankHeight) + "_th&" +
         String(relayOneLimit) + "_l1&" + String(relayTwoLimit) + "_l2&" + String(relayThreeLimit) + "_l3&" +
         String((int)(isRelayOneLowLimit)) + "_s1&" + String((int)(isRelayTwoLowLimit)) + "_s2&" + String((int)(isRelayThreeLowLimit)) + "_s3&" +
         String((int)(relayOneAlarmActive)) + "_a1&" + String((int)(relayTwoAlarmActive)) + "_a2&" + String((int)(relayThreeAlarmActive)) + "_a3&" +
         String(ma4Level) + "_4m&" + String(ma20Level) + "_20m&";
}