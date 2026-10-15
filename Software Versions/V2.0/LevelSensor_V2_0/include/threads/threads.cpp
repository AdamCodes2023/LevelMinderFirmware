#include "LevelSensorV1.h"

using namespace ESPressio::Threads;

class LevelSensorThread : public Thread {
  protected:
    void OnInitialization() override {
      // Anything we need to do here prior to the Thread's Loop starting

      if (isRS485Input) {
        // Initialize Level Sensor Baud Rate
        setLevelSensorBaudRate();

        // Allow some time for initialization
        delay(2500);

        // Initialize Level Sensor Measure Range
        setLevelSensorMeasureRange();

        // Allow some time for initialization
        delay(2500);

        // Initialize Level Sensor Rate Speed
        setLevelSensorRateSpeed();
      }

      // Allow some time for initialization
      delay(1000);
    }

    void OnLoop() override {
      // Whatever we want to do within the Loop
      
      //CHECK LEVEL SENSOR BAUD RATE
      if (!baudRateWriteSuccess && isRS485Input) {
        setLevelSensorBaudRate();
      }

      //CHECK LEVEL SENSOR MEASURE RANGE
      if (!measureRangeWriteSuccess && isRS485Input) {
        setLevelSensorMeasureRange();
      }

      //CHECK LEVEL SENSOR RATE SPEED
      if (!rateSpeedWriteSuccess && isRS485Input) {
        setLevelSensorRateSpeed();
      }

      //POTENTIOMETER TESTING
      //if (!(testOverride || simulationMode)) {
        //sensorValue = readLevelSensorValue420mA();
        //displayIntLong((long)((sensorValue / sensorUnitsPerMeter) * wholeSensorUnitConversionValue));

        //checkRelayOneConditions(sensorValue / sensorUnitsPerMeter);
        //checkRelayTwoConditions(sensorValue / sensorUnitsPerMeter);
        //checkRelayThreeConditions(sensorValue / sensorUnitsPerMeter);
        //setAnalogOutputValue(sensorValue / sensorUnitsPerMeter);
      //}

      //LEVEL SENSOR READING
      currentLevelSensorReadMillis = millis();
      if ((!(testOverride || simulationMode || otaInProgress)) && (currentLevelSensorReadMillis - previousLevelSensorReadMillis >= levelSensorReadWaitInterval)) {
        previousLevelSensorReadMillis = currentLevelSensorReadMillis;
        if (isRS485Input) {
          currentDepth = readLevelSensorValueRS485();
        } else {
          currentDepth = readLevelSensorValue420mA();
        }
      }
      
      delay(10);
    }
};