// A copy of this firmware's license is included in the LICENSE file and is intended to provide for use of this work in not-for-profit applications. Copyleft 2025 by Derek Yerger (CYBORG)

#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BNO055.h>

#define NEOPIXEL_PIN A5
#define NUM_PIXELS 16

Adafruit_BNO055 bno = Adafruit_BNO055(55);

Adafruit_NeoPixel ring(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRBW + NEO_KHZ800);

byte curmode;
long hue = 0;
int brightness = 50; 
byte blinky;
float accelAvg, accelDamping = 0.999, accelWt = 0.2;

float ball_angle = 0.0;  // Ball position in degrees 
float ball_velocity = 0.0;  // Ball velocity 
float damping = 0.95;  // To prevent infinite motion 
float accel_weight = 0.05;  // Scaling factor for acceleration influence

// Handles mapping of input processors to functions. One or more discrete functions are cycled over in a stack and should provide an output, and either jump to/push another state, or exit/pop. 

void setup() {
  Serial.begin(115200);
  if (!bno.begin()) {
    Serial.println("BNO055 not detected");
    while (1);
  } 

  ring.begin();
  ring.clear();
  ring.setBrightness(255);
  ring.show();
}

void loop() {
  //bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  sensors_event_t orientationData , angVelocityData , linearAccelData;
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  double accelSum = abs(linearAccelData.acceleration.x) + abs(linearAccelData.acceleration.y) + abs(linearAccelData.acceleration.z);

  // if ((orientationData.orientation.z < (horiz + horizRg)) && (orientationData.orientation.z > (horiz - horizRg)) && (
  if (accelSum > 0.4) accelAvg += accelSum * accelWt;
  accelAvg *= accelDamping;

  if (curmode == 0) {
    // flashlight when still, blinky rgbw when moving
    byte r = 0, g = 0, b = 0, w = 0, bb = 255;
    bb = min(255, int(accelAvg) * 2);
    blinky = (blinky + 1) % 16; // blinky flashlight
    if (blinky % 4 == 0) ring.clear();
    for (int i = 0; i < 16; i++) {
      if (blinky == 0) w = bb;
      if (blinky == 4) r = bb;
      if (blinky == 8) g = bb;
      if (blinky == 12) b = bb;
    }
    /*if (accelAvg < 1.0) {
      w = 255;
      r = g = b = 0;
    } else */if (accelAvg >= 128.0) accelAvg = 128.0;
    w = max(w, 255 - int(accelAvg * 2));
      //delay(int((accelAvg - 1.0))*5);
    
    ring.fill(ring.Color(r, g, b, w));
    ring.show();
  } else if (curmode == 1) {
    // Compute acceleration influence (Y as forward/backward, X as lateral)
    float accel_x = linearAccelData.acceleration.x * accel_weight;
    float accel_y = linearAccelData.acceleration.z * accel_weight;

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

    if (abs(ball_velocity) < 3) ring.clear(); 
    else {
      hue += 256;
      hue %= 5*65536;
    }
    ring.setPixelColor(NUM_PIXELS - led_index - 1, ring.ColorHSV(hue));
    ring.show();
    delay(5);
  } else if (curmode == 2) {
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    hue = (long) orientationData.orientation.z << 9;
    ring.fill(ring.ColorHSV(hue));
    ring.show();
  }

}
