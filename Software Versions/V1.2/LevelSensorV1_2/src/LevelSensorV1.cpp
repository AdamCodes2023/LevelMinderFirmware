//LEVEL SENSOR TEST PROGRAM
#include "LevelSensorV1.h"

//#include <SPI.h>                                  		//-- moved to arduinoGlue.h
//#include <Arduino.h>                                  //-- moved to arduinoGlue.h

//#include <BLEDevice.h>                            		//-- moved to arduinoGlue.h
//#include <BLEUtils.h>                             		//-- moved to arduinoGlue.h
//#include <BLEServer.h>                            		//-- moved to arduinoGlue.h

// github link: https://github.com/4-20ma/ModbusMaster
//#include <ModbusMaster.h>                         		//-- moved to arduinoGlue.h

//#include <Preferences.h>                          		//-- moved to arduinoGlue.h

//#include <Wire.h>                                 		//-- moved to arduinoGlue.h
//#include <AD56X2.h>                               		//-- moved to arduinoGlue.h

#include "bluetooth/extrabluetooth.cpp"
#include "buttons/buttons.cpp"
#include "display/display.cpp"
#include "io/io.cpp"
#include "modbus/extramodbus.cpp"
#include "threads/threads.cpp"
#include "utilities/utilities.cpp"

// GLOBAL VARIABLES AND CONSTANTS
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

	//-- moved to arduinoGlue.h // #define SEND_CONFIG_SERVICE_UUID            "043d47db-07b6-4112-9505-b8dd9dcd3851"
	//-- moved to arduinoGlue.h // #define SEND_CONFIG_CHARACTERISTIC_UUID     "ea324d87-4867-4ea5-babb-39f1a967bb66"
	//-- moved to arduinoGlue.h // #define DISPLAY_CONFIG_SERVICE_UUID         "4798e66a-1287-48b0-8096-aed974e68b27"
	//-- moved to arduinoGlue.h // #define DISPLAY_CONFIG_CHARACTERISTIC_UUID  "9994a725-36d2-4151-b4fb-204ac9b53ed8"
	//-- moved to arduinoGlue.h // #define SEND_SIMULATION_SERVICE_UUID        "807e4f7a-761d-44f2-9ed9-65b9b97b4abb"
	//-- moved to arduinoGlue.h // #define SEND_SIMULATION_CHARACTERISTIC_UUID "d5dd0354-7334-4ab1-9546-64243e4ebaef"

LevelSensorThread levelSensorModbusThread;

BLEServer *pServer = NULL;
BLECharacteristic *pSendConfigCharacteristic = NULL;
BLECharacteristic *pDisplayConfigCharacteristic = NULL;
BLECharacteristic *pUpdateProgressCharacteristic = NULL;
bool bluetoothActive = false;
bool bleConnected = false;
String macString = String("");

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

/* Modbus Stuff */
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
const int bluetoothWaitInterval = 300000;
unsigned long long previousBluetoothMillis = 0;
unsigned long long currentBluetoothMillis = 0;
const short ledAlarmBlinkInterval = 500;
unsigned long long previousLedAlarmBlinkMillis = 0;
unsigned long long currentLedAlarmBlinkMillis = 0;
const int bluetoothVerifyInterval = 20000;
unsigned long long bluetoothVerifyPreviousMillis = 0;
unsigned long long bluetoothVerifyCurrentMillis = 0;

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

SET_LOOP_TASK_STACK_SIZE(16*1024); // 16KB


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
  node.idle(modbusIdleCPUSafe);

  Wire.begin(i2cDataPin, i2cClockPin);
  //BR = 12;  // 400 kHz (maximum)

  analogOutput.begin();

  BLEDevice::init("RLM-3C");
  BLEDevice::setMTU(BLE_MTU);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  macString = String(BLEDevice::getAddress().toString().c_str());
  //macString.replace(":", "");
  macString.toUpperCase();

  BLEService *pSendConfigService = pServer->createService(SEND_CONFIG_SERVICE_UUID);
  BLEService *pDisplayConfigService = pServer->createService(DISPLAY_CONFIG_SERVICE_UUID);
  BLEService *pSendSimulationService = pServer->createService(SEND_SIMULATION_SERVICE_UUID);
  BLEService *pSendUpdateService = pServer->createService(SEND_UPDATE_SERVICE_UUID);
  pSendConfigCharacteristic = pSendConfigService->createCharacteristic(
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
  resetBluetooth();

  // Initialize Outputs
  digitalWrite(relayOnePin, LOW);
  digitalWrite(relayTwoPin, LOW);
  digitalWrite(relayThreePin, LOW);

  digitalWrite(ledAlarmPin, LOW);

  digitalWrite(alarmHornPin, LOW);

  analogOutput.setOutputLevel((uint16_t)(0));

  // Allow some time for initialization
  delay(500);

  //Start Level Sensor Thread
  levelSensorModbusThread.SetPriority(1UL | portPRIVILEGE_BIT);
  levelSensorModbusThread.SetStackSize(16*1024); // 16KB
  levelSensorModbusThread.Initialize();
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

  //DEBOUNCE BUTTONS
  debounceBluetoothResetButton();
  debounceAlarmTestButton();
  debounceAlarmSilenceButton();

  //Serial.println("BT: " + String(digitalRead(bluetoothResetPin)));
  //Serial.println("Test: " + String(digitalRead(alarmTestPin)));
  //Serial.println("Silence: " + String(digitalRead(alarmSilencePin)));
  //Serial.println("UpInProgress: " + String(otaInProgress));

  //ANALOG INPUT READING
  //if (!(testOverride || simulationMode || otaInProgress)) {
    //Serial.println(String("ANALOG INPUT: ") + String(readAnalogInputValue()));
  //}

  //LED ALARM FLASH
  currentLedAlarmBlinkMillis = millis();
  if (((boardTestMode) || (relayOneAlarmTriggered && relayOneAlarmActive) || (relayTwoAlarmTriggered && relayTwoAlarmActive) || (relayThreeAlarmTriggered && relayThreeAlarmActive)) && (currentLedAlarmBlinkMillis - previousLedAlarmBlinkMillis >= ledAlarmBlinkInterval)) {
    previousLedAlarmBlinkMillis = currentLedAlarmBlinkMillis;
    digitalWrite(ledAlarmPin, ledAlarmOn);
    ledAlarmOn = !ledAlarmOn;
  }

  //BLUETOOTH RECONNECT GRACE PERIOD (20 SECONDS)
  bluetoothVerifyCurrentMillis = millis();
  //if ((bluetoothActive && bleConnected) && (bluetoothVerifyCurrentMillis - bluetoothVerifyPreviousMillis >= bluetoothVerifyInterval)) {
  //  stopBluetooth();
  //}

  //DEACTIVATE BLUETOOTH AFTER INACTIVITY (5 MINUTES)
  currentBluetoothMillis = millis();
  if ((bluetoothActive) && (currentBluetoothMillis - previousBluetoothMillis >= bluetoothWaitInterval)) {
    stopBluetooth();
  }

  if ((bleConnected) && (currentBluetoothMillis - previousBluetoothMillis >= bluetoothWaitInterval * 2)) {
    forceBLEDisconnect();
    //stopBluetooth();
  }
}