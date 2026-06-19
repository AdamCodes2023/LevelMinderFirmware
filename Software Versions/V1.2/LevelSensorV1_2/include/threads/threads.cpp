#include "LevelSensorV1.h"

using namespace ESPressio::Threads;

class LevelSensorThread : public Thread {
  protected:
    void OnInitialization() override {
      // Anything we need to do here prior to the Thread's Loop starting

      // Initialize Level Sensor Baud Rate
      setLevelSensorBaudRate();

      // Allow some time for initialization
      delay(2500);

      // Initialize Level Sensor Measure Range
      setLevelSensorMeasureRange();
    }

    void OnLoop() override {
      // Whatever we want to do within the Loop
      
      //CHECK LEVEL SENSOR BAUD RATE
      if (!baudRateWriteSuccess) {
        setLevelSensorBaudRate();
      }

      //CHECK LEVEL SENSOR MEASURE RANGE
      if (!measureRangeWriteSuccess) {
        setLevelSensorMeasureRange();
      }

      //POTENTIOMETER TESTING
      //if (!(testOverride || simulationMode)) {
        //sensorValue = readLevelSensorValue();
        //displayIntLong((long)((sensorValue / sensorUnitsPerMeter) * wholeSensorUnitConversionValue));

        //checkRelayOneConditions(sensorValue / sensorUnitsPerMeter);
        //checkRelayTwoConditions(sensorValue / sensorUnitsPerMeter);
        //checkRelayThreeConditions(sensorValue / sensorUnitsPerMeter);
        //setAnalogOutputValue(sensorValue / sensorUnitsPerMeter);
      //}

      //MODBUS READING
      currentLevelSensorReadMillis = millis();
      if ((!(testOverride || simulationMode || otaInProgress)) && (currentLevelSensorReadMillis - previousLevelSensorReadMillis >= levelSensorReadWaitInterval)) {
        previousLevelSensorReadMillis = currentLevelSensorReadMillis;
        currentDepth = readLevelSensorValue();
        if (!probeErrorOccurred) {
          displayIntLong((long)(currentDepth / displayAdjustmentFactor));
          checkRelayOneConditions(currentDepth / depthUnitsPerMeter);
          checkRelayTwoConditions(currentDepth / depthUnitsPerMeter);
          checkRelayThreeConditions(currentDepth / depthUnitsPerMeter);
          setAnalogOutputValue(currentDepth / depthUnitsPerMeter);
        } else {
          displayProbeMissingError();
        }
      }
      
      delay(50);
    }
};