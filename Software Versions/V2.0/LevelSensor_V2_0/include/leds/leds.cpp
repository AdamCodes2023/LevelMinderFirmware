#include "LevelSensorV1.h"

#include <driver/rmt.h>

#define NUM_LEDS 25
#define DATA_PIN 40
#define RMT_CHANNEL RMT_CHANNEL_0

#define LEVEL_LEDS 10
#define ALARM_LEDS 5

uint8_t led_data[NUM_LEDS * 3]; // GRB byte array

void initializeLEDs() {
  rmt_config_t config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)DATA_PIN, RMT_CHANNEL);
  config.clk_div = 8; // 80 MHz / 8 = 10 MHz (1 tick = 100 ns)

  rmt_config(&config);
  rmt_driver_install(RMT_CHANNEL, 0, 0);
}

void showLEDs() {
  // Define 100ns tick wave bits matching ~295ns/595ns specs:
  // Bit 0: 3 ticks HIGH (300ns), 6 ticks LOW (600ns)
  // Bit 1: 6 ticks HIGH (600ns), 3 ticks LOW (300ns)
  rmt_item32_t bit0 = {{{ 3, 1, 6, 0 }}};
  rmt_item32_t bit1 = {{{ 6, 1, 3, 0 }}};

  rmt_item32_t items[NUM_LEDS * 24];
  int item_idx = 0;

  for (int i = 0; i < NUM_LEDS * 3; i++) {
    uint8_t byte_val = led_data[i];
    for (int bit = 7; bit >= 0; bit--) {
      if ((byte_val >> bit) & 0x01) {
        items[item_idx++] = bit1;
      } else {
        items[item_idx++] = bit0;
      }
    }
  }

  // Transmit bits over RMT hardware and wait for completion
  rmt_write_items(RMT_CHANNEL, items, NUM_LEDS * 24, true);
  delayMicroseconds(80); // Latch reset pulse (>80us)
}

void setLEDColor(int index, uint8_t r, uint8_t g, uint8_t b) {
  // GRB order
  led_data[index * 3 + 0] = g;
  led_data[index * 3 + 1] = r;
  led_data[index * 3 + 2] = b;
}

void alarmFlashOn(short colorOption) {
  for (int i = 0; i < ALARM_LEDS; i++) {
    switch (colorOption) {
      case 1:
        setLEDColor(i + LEVEL_LEDS, 255, 50, 0); // Solid Orange
        break;
      case 2:
        setLEDColor(i + LEVEL_LEDS, 255, 255, 0); // Solid Yellow
        break;
      default:
        setLEDColor(i + LEVEL_LEDS, 255, 0, 0); // Solid Red
    }
  }

  if (boardTestMode) {
    for (int i = 0; i < LEVEL_LEDS; i++) {
      switch (colorOption) {
        case 1:
          setLEDColor(i, 255, 50, 0); // Solid Orange
          setLEDColor(i + LEVEL_LEDS + ALARM_LEDS, 255, 50, 0); // Solid Orange (Second Column)
          break;
        case 2:
          setLEDColor(i, 255, 255, 0); // Solid Yellow
          setLEDColor(i + LEVEL_LEDS + ALARM_LEDS, 255, 255, 0); // Solid Yellow (Second Column)
          break;
        default:
          setLEDColor(i, 255, 0, 0); // Solid Red
          setLEDColor(i + LEVEL_LEDS + ALARM_LEDS, 255, 0, 0); // Solid Red (Second Column)
      }
    }
  }

  showLEDs();
}

void alarmFlashOff() {
  for (int i = 0; i < ALARM_LEDS; i++) {
    setLEDColor(i + LEVEL_LEDS, 0, 0, 0); // Off
  }

  if (boardTestMode) {
    for (int i = 0; i < LEVEL_LEDS; i++) {
      setLEDColor(i, 0, 0, 0); // Off
      setLEDColor(i + LEVEL_LEDS + ALARM_LEDS, 0, 0, 0); // Off (Second Column)
    }
  }
  
  showLEDs();
}

void setTankLevelLEDs(double level, double fullLevel) {
  int numLitLEDs = (int)((level / fullLevel) * 100.0);
  if (numLitLEDs > 100) numLitLEDs = 100;
  if (numLitLEDs < 0) numLitLEDs = 0;

  for (int i = 0; i < LEVEL_LEDS; i++) {
    if ((i * 10) + 1 <= numLitLEDs) {
      if (relayOneAlarmTriggered && (i * 10) + 1 >= (int)(((relayOneLimit - emptyLevel) / fullLevel) * 100.0) - 10) {
        setLEDColor(i, 255, 0, 0); // Red For High Liquid Level
        setLEDColor(i + LEVEL_LEDS + ALARM_LEDS, 255, 0, 0); // Red (Second Column)
      } else if (relayTwoAlarmTriggered) {
        setLEDColor(i, 255, 255, 0); // Yellow For Low Liquid Level
        setLEDColor(i + LEVEL_LEDS + ALARM_LEDS, 255, 255, 0); // Yellow (Second Column)
      }
      else {
        setLEDColor(i, 0, 0, 255); // Blue For Liquid Level
        setLEDColor(i + LEVEL_LEDS + ALARM_LEDS, 0, 0, 255); // Blue (Second Column)
      }
    } else if (numLitLEDs == 0 && i == 0) {
      setLEDColor(i, 255, 255, 0); // Yellow For Empty Tank Level
      setLEDColor(i + LEVEL_LEDS + ALARM_LEDS, 255, 255, 0); // Yellow (Second Column)
    } else {
      setLEDColor(i, 0, 0, 0); // Off For No Liquid
      setLEDColor(i + LEVEL_LEDS + ALARM_LEDS, 0, 0, 0); // Off (Second Column)
    }
  }
  showLEDs();
}

void setProbeMissingLEDs() {
  static short ledIndex = 0;
  static bool ledDirection = true; // true = forward, false = backward

  static unsigned long long previousLedMoveMillis = 0;
  static unsigned long long currentLedMoveMillis = 0;

  currentLedMoveMillis = millis();
  if (currentLedMoveMillis - previousLedMoveMillis >= 250) {
    previousLedMoveMillis = currentLedMoveMillis;

    for (int i = 0; i < LEVEL_LEDS; i++) {
      setLEDColor(i, 0, 0, 0); // Off
      setLEDColor(i + LEVEL_LEDS + ALARM_LEDS, 0, 0, 0); // Off (Second Column)
    }
    for (int i = 0; i < ALARM_LEDS; i++) {
      setLEDColor(i + LEVEL_LEDS, 0, 0, 0); // Off
    }

    setLEDColor(ledIndex + LEVEL_LEDS, 255, 50, 0); // Orange
    setLEDColor(ledIndex + LEVEL_LEDS + 1, 255, 50, 0); // Orange (Second Light)

    showLEDs();

    //Serial.println("Probe Missing LED Index: " + String(ledIndex));

    // Move LEDs for the next iteration (Knight Rider Style).
    if (ledDirection) {
      ledIndex++;
      if (ledIndex >= ALARM_LEDS - 2) {
        ledDirection = false;
      }
    } else {
      ledIndex--;
      if (ledIndex <= 0) {
        ledDirection = true;
      }
    }
  }
}

void setStartUpLEDs() {
  // Slowly fill the two columns of level LEDs with green color for startup animation.
  for (int i = 0; i < LEVEL_LEDS; i++) {
    setLEDColor(i, 0, 255, 0); // Green For Startup Animation
    setLEDColor(i + LEVEL_LEDS + ALARM_LEDS, 0, 255, 0); // Green (Second Column)
    delay(200);
    showLEDs();
  }

  // Set the alarm LEDs to green for startup animation.
  delay(100);
  for (int i = 0; i < ALARM_LEDS; i++) {
    setLEDColor(i + LEVEL_LEDS, 0, 255, 0); // Green For Startup Animation
  }
  showLEDs();
}