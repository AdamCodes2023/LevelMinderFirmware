#ifndef ARDUINOGLUE_H
#define ARDUINOGLUE_H


//============ Includes ====================
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ModbusMaster.h>
#include <Preferences.h>
#include <Wire.h>
#include <AD56X2.h>
#include <Update.h>

//============ Defines & Macros====================
#define FIRMWARE_VERSION                    "V 1.1.1"
#define BLE_MTU 240
#define SEND_CONFIG_SERVICE_UUID            "043d47db-07b6-4112-9505-b8dd9dcd3851"
#define SEND_CONFIG_CHARACTERISTIC_UUID     "ea324d87-4867-4ea5-babb-39f1a967bb66"
#define DISPLAY_CONFIG_SERVICE_UUID         "4798e66a-1287-48b0-8096-aed974e68b27"
#define DISPLAY_CONFIG_CHARACTERISTIC_UUID  "9994a725-36d2-4151-b4fb-204ac9b53ed8"
#define SEND_SIMULATION_SERVICE_UUID        "807e4f7a-761d-44f2-9ed9-65b9b97b4abb"
#define SEND_SIMULATION_CHARACTERISTIC_UUID "d5dd0354-7334-4ab1-9546-64243e4ebaef"
#define SEND_UPDATE_SERVICE_UUID            "6c8b91e1-5e15-40a4-8be8-477b5dda227b"
#define SEND_UPDATE_CHARACTERISTIC_UUID     "344e3d5a-da87-4321-b887-5e18f6c7f56a"
#define UPDATE_PROGRESS_CHARACTERISTIC_UUID "a93637e4-b18f-486a-b169-c3affb9a98cc"
#define MODBUS_RXE_PIN 1 // RxEn pin
#define MODBUS_TXE_PIN 2 // TxEn pin
#define MODBUS_RX_PIN 44 // Rx pin
#define MODBUS_TX_PIN 43 // Tx pin
#define MODBUS_SERIAL_BAUD 9600 // Baud rate for esp32 and max485 communication

//============ Extern Variables ============
extern int             alarmHornPin;                      		//-- from LevelSensorV1
extern bool            alarmSilenceButtonPressed;         		//-- from LevelSensorV1
extern int             alarmSilencePin;                   		//-- from LevelSensorV1
extern bool            alarmTestButtonPressed;            		//-- from LevelSensorV1
extern int             alarmTestPin;                      		//-- from LevelSensorV1
extern int             analogInputPin;                    		//-- from LevelSensorV1
extern int             analogInputValue;                  		//-- from LevelSensorV1
extern AD56X2          analogOutput;                      		//-- from LevelSensorV1
extern int             analogOutputI2cAddress;            		//-- from LevelSensorV1
extern bool            baudRateWriteSuccess;              		//-- from LevelSensorV1
extern uint16_t        baud_register;                     		//-- from LevelSensorV1
extern bool            bluetoothActive;                   		//-- from LevelSensorV1
extern bool            bluetoothResetButtonPressed;       		//-- from LevelSensorV1
extern int             bluetoothResetPin;                 		//-- from LevelSensorV1
extern const int       bluetoothWaitInterval;             		//-- from LevelSensorV1
extern bool            boardTestMode;                     		//-- from LevelSensorV1
extern int             currentDepth;                      		//-- from LevelSensorV1
extern const double    depthUnitsPerMeter;                		//-- from LevelSensorV1
extern const int       displayAdjustmentFactor;           		//-- from LevelSensorV1
extern int             displayClockPin;                   		//-- from LevelSensorV1
extern int             displayDataPin;                    		//-- from LevelSensorV1
extern int             displayEnablePin;                  		//-- from LevelSensorV1
extern double          emptyLevel;                        		//-- from LevelSensorV1
extern String          emptyLevelKey;                     		//-- from LevelSensorV1
extern const double    feetToDepthUnitsConversionFactor;  		//-- from LevelSensorV1
extern const double    feetToMetersConversionFactor;      		//-- from LevelSensorV1
extern double          fullLevel;                         		//-- from LevelSensorV1
extern String          fullLevelKey;                      		//-- from LevelSensorV1
extern int             i2cClockPin;                       		//-- from LevelSensorV1
extern int             i2cDataPin;                        		//-- from LevelSensorV1
extern bool            isRelayOneLowLimit;                		//-- from LevelSensorV1
extern String          isRelayOneLowLimitKey;             		//-- from LevelSensorV1
extern bool            isRelayThreeLowLimit;              		//-- from LevelSensorV1
extern String          isRelayThreeLowLimitKey;           		//-- from LevelSensorV1
extern bool            isRelayTwoLowLimit;                		//-- from LevelSensorV1
extern String          isRelayTwoLowLimitKey;             		//-- from LevelSensorV1
extern bool            isUnitMeters;                      		//-- from LevelSensorV1
extern String          isUnitMetersKey;                   		//-- from LevelSensorV1
extern const short     ledAlarmBlinkInterval;             		//-- from LevelSensorV1
extern bool            ledAlarmOn;                        		//-- from LevelSensorV1
extern int             ledAlarmPin;                       		//-- from LevelSensorV1
extern int             levelSensorModbusID;               		//-- from LevelSensorV1
extern const short     levelSensorReadWaitInterval;       		//-- from LevelSensorV1
extern uint16_t        level_register;                    		//-- from LevelSensorV1
extern double          ma20Level;                         		//-- from LevelSensorV1
extern String          ma20LevelKey;                      		//-- from LevelSensorV1
extern double          ma4Level;                          		//-- from LevelSensorV1
extern String          ma4LevelKey;                       		//-- from LevelSensorV1
extern bool            measureRangeWriteSuccess;          		//-- from LevelSensorV1
extern const double    metersToDepthUnitsConversionFactor;		//-- from LevelSensorV1
extern const double    metersToFeetConversionFactor;      		//-- from LevelSensorV1
extern ModbusMaster    node;                              		//-- from LevelSensorV1
extern String          pnamespace;                        		//-- from LevelSensorV1
extern Preferences     preferences;                       		//-- from LevelSensorV1
extern int             probeErrorCount;                   		//-- from LevelSensorV1
extern bool            probeErrorOccurred;                		//-- from LevelSensorV1
extern uint16_t        range_register;                    		//-- from LevelSensorV1
extern bool            relayOneAlarmActive;               		//-- from LevelSensorV1
extern String          relayOneAlarmActiveKey;            		//-- from LevelSensorV1
extern bool            relayOneAlarmTriggered;            		//-- from LevelSensorV1
extern double          relayOneLimit;                     		//-- from LevelSensorV1
extern String          relayOneLimitKey;                  		//-- from LevelSensorV1
extern int             relayOnePin;                       		//-- from LevelSensorV1
extern bool            relayThreeAlarmActive;             		//-- from LevelSensorV1
extern String          relayThreeAlarmActiveKey;          		//-- from LevelSensorV1
extern bool            relayThreeAlarmTriggered;          		//-- from LevelSensorV1
extern double          relayThreeLimit;                   		//-- from LevelSensorV1
extern String          relayThreeLimitKey;                		//-- from LevelSensorV1
extern int             relayThreePin;                     		//-- from LevelSensorV1
extern bool            relayTwoAlarmActive;               		//-- from LevelSensorV1
extern String          relayTwoAlarmActiveKey;            		//-- from LevelSensorV1
extern bool            relayTwoAlarmTriggered;            		//-- from LevelSensorV1
extern double          relayTwoLimit;                     		//-- from LevelSensorV1
extern String          relayTwoLimitKey;                  		//-- from LevelSensorV1
extern int             relayTwoPin;                       		//-- from LevelSensorV1
extern bool            silenceOverride;                   		//-- from LevelSensorV1
extern bool            simulationMode;                    		//-- from LevelSensorV1
extern double          tankHeight;                        		//-- from LevelSensorV1
extern String          tankHeightKey;                     		//-- from LevelSensorV1
extern bool            testOverride;                      		//-- from LevelSensorV1
extern bool            otaInProgress;
extern uint32_t        otaFileSize;
extern uint32_t        otaReceived;
extern int             lastProgressPercent;
extern bool            fwWriteFlag;

//============ Function Prototypes =========
//-- from LevelSensorV1.ino -----------
void displayOn();                                           
void displayOff();                                          
void displayTest1();                                        
void displayTest2();                                        
void displayIntLong(long x);                                
void displayProbeMissingError();                            
void setLevelSensorBaudRate();                              
void setLevelSensorMeasureRange();                          
int readAnalogInputValue();                                 
int readLevelSensorValue();                                 
void checkRelayOneConditions(double levelSensorValue);      
void checkRelayTwoConditions(double levelSensorValue);      
void checkRelayThreeConditions(double levelSensorValue);    
void setAnalogOutputValue(double levelSensorValue);         
void onConnect(BLEServer* pServer);                         
void onDisconnect(BLEServer* pServer);                      
void onWrite(BLECharacteristic *pCharacteristic);           
void modbusPreTransmission();                               
void modbusPostTransmission();                              
void storeConfigValue(String key, String value);            
String findConfigValue(String key, String defaultValue = String());
void debounceBluetoothResetButton();                        
void debounceAlarmTestButton();                             
void debounceAlarmSilenceButton();                          
void boardTestSequence();                                   
void resetBoard();                                          
void resetBluetooth();                                      
void stopBluetooth();                                       
String createDisplayConfigString();                         
void rebootEspWithReason(String reason);      
void hexCharacterStringToBytes(byte *byteArray, const char *hexString);   
byte nibble(char c);           

#endif // ARDUINOGLUE_H
