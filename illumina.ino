// A copy of this firmware's license is included in the LICENSE file and is intended to provide for use of this work in not-for-profit applications. Copyleft 2025 by Derek Yerger (CYBORG)

#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BNO055.h>
#include <EEPROM.h>

#define NEOPIXEL_PIN A5
#define NUM_PIXELS 16

Adafruit_BNO055 bno = Adafruit_BNO055(55);

Adafruit_NeoPixel ring(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRBW + NEO_KHZ800);

unsigned long ultimer, ultimer2;


byte curmode; /* mode 0: off until */ const int oriNadir = -70; /* for time
interval */ const int nadirTime = 500; /* then gyro lift to horizontal
defined by */ const int raiseTime = 250; /* and */ const int oriHoriz = -10;

/* Then mode 1 is gravity ball mode, note the */ float ball_angle = 0.0; /* ball
position in degrees, as well as the */ float ball_velocity = 0.0; /* Modify these
using a */ float damping = 0.95; /* factor to prevent infinite motion, and
contribute to the change according to a */ float accel_weight = 0.2; /* scaling
factor for acceleration influence. Side sway makes the white pixel go side to
side and turns off/mode 0 if going to nadir. If linear accel exceeds */ const
float ballToFlashlightAccel = 15.0; /* then go round the ring, lighting up the
whole thing and onto mode 3 with fading between solid white and flashing */


long hue = 0;
byte speedMix = 0; /* slowing forward movement blends towards solid white,
                      speeding up toward flashy */
const float accelMin = 0.5;
const byte accelWtInc = 5;
byte oriMix = 0; /* lights off at nadir fade to on
at */ const int oriIlluminated = -10;
byte blinky;
float accelAvg, accelDamping = 0.999;

// Handles mapping of input processors to functions. One or more discrete functions are cycled over in a stack and should provide an output, and either jump to/push another state, or exit/pop. 

void setup() {
  Serial.begin(115200);
  if (!bno.begin()) {
    Serial.println("BNO055 not detected");
    while (1);
  } 
  int eeAddress = 0;
  long bnoID;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  bno.getSensor(&sensor);
  if (bnoID == sensor.sensor_id) {
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);
    bno.setSensorOffsets(calibrationData);
  }

  ring.begin();
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
  bv = constrain(bv, oriNadir, oriIlluminated);
  bv = map(bv, oriNadir, oriIlluminated, 0, 255);
  ring.setBrightness(bv);
  ring.show();

  if (curmode == 0) {
    if (orientationData.orientation.z >= oriNadir) ultimer = millis();
    else if ((millis() - ultimer) > nadirTime) ultimer2 = millis();
    if ((orientationData.orientation.z > oriHoriz) && ((millis() - ultimer2) < raiseTime)) curmode++;
    ring.clear();
    ring.show();
  }
  if (curmode == 1) {
    // Compute acceleration influence (Y as forward/backward, X as lateral)
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    float accel_x = accel.x() * accel_weight;
    float accel_y = accel.z() * accel_weight;

    // Gravity naturally pulls the ball downward (towards 180 degrees)
    float gravity_effect = 0.001; // Adjust as needed
    if (ball_angle < 180) ball_velocity += gravity_effect;
    else ball_velocity -= gravity_effect;

    // Update velocity based on acceleration
    ball_velocity += accel_x * cos(ball_angle * DEG_TO_RAD) + accel_y * sin(ball_angle * DEG_TO_RAD);
    ball_velocity *= damping;

    // Update ball position
    ball_angle += ball_velocity;
    if (ball_angle < 0) ball_angle += 360;
    if (ball_angle >= 360) ball_angle -= 360;

    // Map angle to NeoPixel ring (16 LEDs, 360 degrees)
    int led_index = round((ball_angle / 360.0) * NUM_PIXELS + NUM_PIXELS/2) % NUM_PIXELS;

    if (abs(ball_velocity) < ballToFlashlightAccel) ring.clear();
    else {
      for (int i=0; i<NUM_PIXELS; i++) {
        ring.setPixelColor((NUM_PIXELS - led_index - 1 + 5 + i) % NUM_PIXELS, ring.Color(0, 0, 0, 255));
        ring.show();
        delay(5);
      }
      curmode++;
    }
    /*else {
      hue += 256;
      hue %= 5*65536;
    }
    ring.setPixelColor(NUM_PIXELS - led_index - 1, ring.ColorHSV(hue));*/
    ring.setPixelColor((NUM_PIXELS - led_index + 5) % NUM_PIXELS, ring.Color(0, 0, 0, 255));
    ring.show();
    delay(5);
  }
  if (curmode == 2) {
    // flashlight when still, blinky rgbw when moving
    if ((speedMix < 255) && (linearAccelData.acceleration.y < -accelMin)) speedMix += accelWtInc;
    if ((speedMix > 0) && (linearAccelData.acceleration.y > accelMin)) speedMix -= accelWtInc;
    speedMix = min(255, speedMix);
    speedMix = max(0, speedMix);

    byte r = 0, g = 0, b = 0, w = 0, bb = speedMix;
    blinky = (blinky + 1) % 16; // blinky flashlight
    if (blinky % 4 == 0) ring.clear();
    for (int i = 0; i < 16; i++) {
      if (blinky == 0) w = bb;
      if (blinky == 4) r = bb;
      if (blinky == 8) g = bb;
      if (blinky == 12) b = bb;
    }
    w = max(w, 255 - speedMix);
      //delay(int((accelAvg - 1.0))*5);
    
    ring.fill(ring.Color(r, g, b, w));
    ring.show();
    if (orientationData.orientation.z >= oriNadir) ultimer = millis();
    else if ((millis() - ultimer) > nadirTime) curmode = 0;
  } else if (curmode == 3) {
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    hue = (long) orientationData.orientation.z << 9;
    ring.fill(ring.ColorHSV(hue));
    ring.show();
  }

}
