#include <Wire.h>
#include <MPU6050.h>
#include <FastLED.h>
#include <math.h>
#include "PixelMap.h"



class TwinkleEffect {

    public:

        void update(LEDMap *ledMap) {
            for(int i = 0; i < NUM_LEDS; i++) {
                if(!ledMap->colors[i]) {
                    ledMap->colors[i] = CRGB(255, 255, 255);
                } else {
                    ledMap->colors[i]--;
                }
            }
        }
};
DEFINE_GRADIENT_PALETTE( heatmap_gp ) {
  0,     0,  0,  0,   //black
128,   0,0,  255,   //green
200,   255,0,  255,   //bright yellow
255,   255,255,255 }; //full white

CRGBPalette256 myPal = heatmap_gp;

class RadarSweepEffect {

    public:
        uint8_t LED_color_indices[NUM_LEDS] = {0};

        void update(LEDMap *ledMap, int16_t angle) {
            angle = (angle + 1)%360;
            for(int i = 0; i<NUM_LEDS; i++){
                int16_t dist = ledMap->getThetaDistance(i, angle);
                dist = map(dist, 0, 20, 255, 0);
                if(dist < 0){
                    dist = 0;
                }

                if(LED_color_indices[i] > 10){
                    LED_color_indices[i] -= 10;
                } else {
                    LED_color_indices[i] = 0;
                }
                if(dist > LED_color_indices[i]) {
                    LED_color_indices[i] = dist;
                }
                ledMap->colors[i] = ColorFromPalette(myPal, LED_color_indices[i]);
//                ledMap->colors[i].fadeToBlackBy(20);
//                if(CRGB(dist, dist, dist) > ledMap->colors[i]){
//                    ledMap->colors[i] = CRGB(dist, dist, dist);
//                }
            }
        }
};

class Torque {
    public:
        void update(int16_t);
        int16_t velocity;
        float torque;
};

void Torque::update(int16_t new_vel){
    int16_t new_torque = new_vel - velocity;
    velocity = new_vel;
//    torque = new_torque;
    torque += (0.1) * (new_torque - torque);
}

class Spinner {
    public:
    Spinner();
    float angle;
    float velocity;
    update(float, int16_t);

};

Spinner::Spinner(void){
    angle = 0;
    velocity = 0;
}
Spinner::update(float torque, int16_t gyro_velocity){
    float div;
    if(torque * gyro_velocity > 0 && abs(gyro_velocity) > 100) { // If torque and velocity are in the same direction
        velocity += torque / 10.0; // Increase velocity by torque
    }
    angle += velocity / 20;
    div = angle / 360.0;
    div = floor(div);
    angle = angle - (div*360);

    if(velocity > 0){
        velocity -= 1;
    } else {
        velocity += 1;
    }
}

LEDMap leds;
TwinkleEffect twinkleEffect;
RadarSweepEffect radarSweepEffect;
Torque torque;
Spinner spin;
MPU6050 mpu;

void setup() {

    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds.colors, NUM_LEDS);
    Serial.println("Initialized LEDs");

    // MPU setup
    Serial.begin(115200);
    Serial.println("Initialized MPU6050");

    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        delay(500);
    }
    mpu.calibrateGyro();
    mpu.setThreshold(3);

    FastLED.clear();

}

void loop() {
//    twinkleEffect.update(&leds);
    Vector rawGyro = mpu.readRawGyro();
    torque.update(rawGyro.XAxis);
    spin.update(torque.torque, rawGyro.XAxis);
    radarSweepEffect.update(&leds, spin.angle);
//    Serial.print(spin.angle);
//    Serial.print(" ");
//    Serial.println(spin.velocity);
//    Serial.println(static_cast<int16_t>(rawGyro.XAxis));
    FastLED.show();
    delay(10);
}
