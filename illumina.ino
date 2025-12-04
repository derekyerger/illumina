// A copy of this firmware's license is included in the LICENSE file and is intended to provide for use of this work in not-for-profit applications. Copyleft 2025 by Derek Yerger (CYBORG)

#include "illumina.h"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

Adafruit_NeoPixel ring(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRBW + NEO_KHZ800);

unsigned long ultimer, ultimer2;
byte curmode;
float modeBearing;
long hue = 0;
int speedMix = 0; /* slowing forward movement blends towards solid white,
                      speeding up toward flashy */
float ball_angle = 0.0;
float ball_velocity = 0.0;

#define MAGIC 42069
illumina_ctrl_t curSettings = defSettings;

byte colorScheme = 0;
int csOri1 = -60;
int csOri2 = 10;

byte oriMix = 0;
byte blinky;
float accelAvg, accelDamping = 0.999;

int delayCtr;
byte delayDecay = 2;
int delayOri = 60;
byte delayMax = 1;

byte textPtr;
char text[17] = "cyborg will rise";
const byte textSz = 16;

// Handles mapping of input processors to functions. One or more discrete functions are cycled over in a stack and should provide an output, and either jump to/push another state, or exit/pop. 

void bleDisconnect() {
  ble.sendCommandCheckOK("AT+GAPCONNECTABLE=0");
  ble.sendCommandCheckOK("AT+GAPDISCONNECT");
  ble.sendCommandCheckOK("AT+GAPSTOPADV");
}

void bleConnect() {
  ble.echo(false);
  ble.sendCommandCheckOK("AT+BleHIDEn=On");
  ble.sendCommandCheckOK("AT+BleKeyboardEn=On");
  ble.sendCommandCheckOK("AT+GAPCONNECTABLE=1");
  ble.sendCommandCheckOK("AT+GAPSTARTADV");
  ble.sendCommandCheckOK("AT+BLEKEYBOARDCODE=00-00");
  ble.reset();
}

void setup() {
  Serial.begin(115200);
  ring.begin();
  if (!bno.begin()) {
    Serial.println("BNO055 not detected");
    ring.fill(ring.Color(255,0,0));
    ring.show();
    while (1);
  } 

  ble.begin(0);
  //bleDisconnect();
  if ( FACTORYRESET_ENABLE ) {
    ble.factoryReset();
    ble.sendCommandCheckOK("AT+GAPDEVNAME=cyborg");
    ble.sendCommandCheckOK("AT+HWMODELED=DISABLE");
  }
  bleDisconnect();
  ble.setMode(BLUEFRUIT_MODE_DATA);

  int eeAddress = 0;
  long bnoID;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  bno.getSensor(&sensor);
  eeAddress += sizeof(long);
  if (bnoID == sensor.sensor_id) {
    EEPROM.get(eeAddress, calibrationData);
    bno.setSensorOffsets(calibrationData);
  }
  eeAddress += sizeof(adafruit_bno055_offsets_t);

  uint16_t mag;
  EEPROM.get(eeAddress, mag);
  if (mag == MAGIC) {
    eeAddress += sizeof(uint16_t);
    EEPROM.get(eeAddress, curSettings);
  } else {
    mag = MAGIC;
    EEPROM.put(eeAddress, mag);
    eeAddress += sizeof(uint16_t);
    EEPROM.put(eeAddress, curSettings);
  }

  ring.clear();
  ring.show();
}

void loop() {
  //bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  sensors_event_t orientationData , angVelocityData , linearAccelData;
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  double accelSum = abs(linearAccelData.acceleration.x) + abs(linearAccelData.acceleration.y) + abs(linearAccelData.acceleration.z);

  // if ((orientationData.orientation.z < (horiz + horizRg)) && (orientationData.orientation.z > (horiz - horizRg)) && (
  int bv = orientationData.orientation.z;
  bv = constrain(bv, curSettings.oriNadir, curSettings.oriIlluminated);
  bv = map(bv, curSettings.oriNadir, curSettings.oriIlluminated, 0, 255);
  ring.setBrightness(bv);
  ring.show();

  if (curmode == 0) {
    if (orientationData.orientation.z >= curSettings.oriNadir) ultimer = millis();
    else if ((millis() - ultimer) > curSettings.nadirTime) {
      ultimer2 = millis();
      modeBearing = orientationData.orientation.x;
    }
    if ((orientationData.orientation.z > curSettings.oriHoriz) && ((millis() - ultimer2) < curSettings.raiseTime)) {
      // Select next mode based on bearing
      if ((orientationData.orientation.x - modeBearing) < curSettings.bearingBoundL)
        curmode = 5;
      else if ((orientationData.orientation.x - modeBearing) > curSettings.bearingBoundR)
        curmode = 6;
      else curmode++;
    }
    ring.clear();
    ring.show();
  }
  if (curmode == 1) {
    // Compute acceleration influence (Y as forward/backward, X as lateral)
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    float accel_x = accel.x() * curSettings.m1_accel_weight;
    float accel_y = accel.z() * curSettings.m1_accel_weight;

    // Gravity naturally pulls the ball downward (towards 180 degrees)
    if (ball_angle < 180) ball_velocity += curSettings.m1_gravity_effect;
    else ball_velocity -= curSettings.m1_gravity_effect;

    // Update velocity based on acceleration
    ball_velocity += accel_x * cos(ball_angle * DEG_TO_RAD) + accel_y * sin(ball_angle * DEG_TO_RAD);
    ball_velocity *= curSettings.m1_damping;

    // Update ball position
    ball_angle += ball_velocity;
    if (ball_angle < 0) ball_angle += 360;
    if (ball_angle >= 360) ball_angle -= 360;

    // Map angle to NeoPixel ring (16 LEDs, 360 degrees)
    int led_index = round((ball_angle / 360.0) * NUM_PIXELS + NUM_PIXELS/2) % NUM_PIXELS;

    if (abs(ball_velocity) < curSettings.m1_ballToFlashlightAccel) ring.clear();
    else if ((millis() - ultimer) > curSettings.m1_ballSettleTime) {
      for (int i=0; i<NUM_PIXELS; i++) {
        ball_angle += ball_velocity;
        if (ball_angle < 0) ball_angle += 360;
        if (ball_angle >= 360) ball_angle -= 360;
        led_index = round((ball_angle / 360.0) * NUM_PIXELS + NUM_PIXELS/2) % NUM_PIXELS;
        ring.setPixelColor((led_index + curSettings.m1_ringOffset) % NUM_PIXELS, ring.Color(0, 0, 0, 255));
        ring.show();
        delay(curSettings.m2_ballTrDelay);
      }
      curmode++;
    }
    /*else {
      hue += 256;
      hue %= 5*65536;
    }
    ring.setPixelColor(NUM_PIXELS - led_index - 1, ring.ColorHSV(hue));*/
    ring.setPixelColor((led_index + curSettings.m1_ringOffset) % NUM_PIXELS, ring.Color(0, 0, 0, 255));
    ring.show();
    delay(5);
  }
  if (curmode == 2) {
    // flashlight when still, blinky rgbw when moving
    if (orientationData.orientation.z > delayOri) 
      delayCtr = max(delayCtr, (orientationData.orientation.z - delayOri) * 2);
    else delayCtr = max(0, delayCtr - delayDecay);
    if (delayCtr) {
      delay(delayCtr);
      speedMix = 255;
    }
    if (accelSum > curSettings.m2_accelMin) speedMix += curSettings.m2_accelWtInc;
    else speedMix--;
    if ((abs(angVelocityData.gyro.x) > curSettings.m2_gyroMin) || (abs(angVelocityData.gyro.y) > curSettings.m2_gyroMin)) speedMix += curSettings.m2_gyroWtInc;
    speedMix = constrain(speedMix - 1, 0, 255);

    if (orientationData.orientation.z < csOri1) {
      colorScheme = 4;
      textPtr = 1;
      delay(delayMax);
    } else if (colorScheme == 4) {
      colorScheme = 3;
      blinky = 15;
    }
    if (textPtr > 0) {
      speedMix = 255;
      //Serial.println(textPtr);
    } else {
      if (orientationData.orientation.z > csOri2) colorScheme = 1;
      else colorScheme = 0;
    }

    byte r = 0, g = 0, b = 0, w = 0, bb = speedMix;
    blinky = (blinky + 1) % 16; // blinky flashlight
    if (blinky % 4 == 0) ring.clear();
    //for (int i = 0; i < 16; i++) {
      if (colorScheme <= 2) {
        if (!delayCtr && (blinky < 4)) blinky = 4;
        if (blinky == 0) w = bb;
        //if (!delayCtr && ((blinky % 4) == 0)) w = bb;
        if (blinky == 4) r = bb;
        if (!delayCtr && ((blinky % 4) == 1)) r = bb;
        if (blinky == 8) g = bb;
        if (!delayCtr && ((blinky % 4) == 2)) g = bb;
        if (blinky == 12) b = bb;
        if (!delayCtr && ((blinky % 4) == 3)) b = bb;
      }
      if (colorScheme == 1) {
        if (blinky == 4) g = bb;
        if (!delayCtr && ((blinky % 4) == 1)) g = bb;
        if (blinky == 8) b = bb;
        if (!delayCtr && ((blinky % 4) == 2)) b = bb;
        if (blinky == 12) r = bb;
        if (!delayCtr && ((blinky % 4) == 3)) r = bb;
      }
      if (colorScheme == 2) {
        if (blinky == 2) r = bb;
        if (blinky == 6) g = bb;
        if (blinky == 10) b = bb;
        if (blinky == 4) g = bb;
        if (blinky == 8) b = bb;
        if (blinky == 12) r = bb;
      }
      if (colorScheme == 3) {
        if (text[textPtr - 1] & (blinky >> 1)) b = 255;
        //else r = 255;
        /* if ((blinky % 2) == 1) {
          r = b;
          b = 0;
        }*/
        if (blinky == 0) w = 255;
        if (blinky == 15) textPtr = (textPtr + 1) % (textSz + 1);
    /*    Serial.print(textPtr);
        Serial.print(' ');
        Serial.print(text[textPtr - 1]);
        Serial.print(' ');
        Serial.println(text[textPtr - 1] & (blinky >> 1));
        delay(20);*/
      }
      if (colorScheme == 4) {
        /*if (blinky < 8) {*/ r = 255; g = 96; /*}
        else b = 255;*/
      }
   // }
    w = max(w, 255 - speedMix);
      //delay(int((accelAvg - 1.0))*5);
    
    ring.fill(ring.Color(r, g, b, w));
    ring.show();
    if (orientationData.orientation.z >= curSettings.oriNadir) ultimer = millis();
    else if ((millis() - ultimer) > curSettings.nadirTime) curmode = 0;
  } else if (curmode == 5) {
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    hue = (long) orientationData.orientation.z << 9;
    ring.fill(ring.ColorHSV(hue));
    ring.show();
    if (orientationData.orientation.z >= curSettings.oriNadir) ultimer = millis();
    else if ((millis() - ultimer) > curSettings.nadirTime) curmode = 0;
  } else if (curmode == 6) {
    byte r = 0, g = 0, b = 0, w = 0;
    if (orientationData.orientation.z > delayOri)
      w = map(orientationData.orientation.z, delayOri, 90, 0, 255); 
    
    if (orientationData.orientation.z > -35) blinky = (blinky + 1) % 6; // random color of RGBCMY

    if (blinky % 3 == 0) r = 255;
    else if (blinky % 3 == 1) g = 255;
    else if (blinky % 3 == 2) b = 255;
    switch (blinky) {
      case 3: g = 255; break;
      case 4: b = 255; break;
      case 5: r = 255;
    }
    
    ring.fill(ring.Color(r, g, b, w));
    ring.show();
    if (orientationData.orientation.z >= curSettings.oriNadir) ultimer = millis();
    else if ((millis() - ultimer) > curSettings.nadirTime) curmode = 0;
    delay(5);
  }
  handleAdjust();
}

void handleAdjust() {
  while (ble.available()) {
      byte val = ble.read();
      if (val == '0') {
        curmode = 0;
        ble.println("current mode 0");
      }
      if (val == '5') {
        curmode = 5;
        ble.println("current mode 5");
      }
    }
}
