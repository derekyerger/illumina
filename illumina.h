#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BNO055.h>

#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include "BluefruitConfig.h"
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif
#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

/** Lighting-specific settings **/
#define NEOPIXEL_PIN A5
#define NUM_PIXELS 16

/** A structure to represent tunable settings associated with operating modes */
typedef struct {
  int16_t oriNadir; // Pitch for pointed at ground
  int16_t nadirTime; // How long here till modeswitch
  int16_t raiseTime; // How long to transit to horizontal for modeswitch
  int16_t oriHoriz; // Pitch to consider modeswitch
  int16_t bearingBoundL; // Bearing travel left for different mode
  int16_t bearingBoundR; // Bearing travel left for different mode
  int16_t oriIlluminated; // From nadir to here fades in
  
  /* Mode 1: gravity ball */
  float m1_gravity_effect;
  float m1_damping;
  float m1_accel_weight;
  uint8_t m1_ballToFlashlightAccel;
  uint16_t m1_ballSettleTime;
  uint8_t m1_ringOffset;

  /* Mode 2: flashlight with movement component */
  float m2_accelMin;
  uint8_t m2_accelWtInc;
  float m2_gyroMin;
  uint8_t m2_gyroWtInc;
  uint8_t m2_ballTrDelay;
} illumina_ctrl_t;

const illumina_ctrl_t defSettings = {-75, 500, 250, -10, -30, 0, -10, 
  0.001, 0.95, 0.2, 15, 500, 7,
  4.0, 10, 3.0, 10, 25 };
