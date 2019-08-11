#include <Wire.h>
#include <MPU6050.h>
#include <FastLED.h>
#include "PixelMap.h"


MPU6050 mpu;

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

class RadarSweepEffect {
    uint16_t angle;

    public:
        RadarSweepEffect(void);
        void update(LEDMap *ledMap) {
            angle = (angle + 1)%360;
            for(int i = 0; i<NUM_LEDS; i++){
                int16_t dist = ledMap->getThetaDistance(i, angle);
                dist = map(dist, 0, 20, 255, 0);
                if(dist < 0){
                    dist = 0;
                }
                ledMap->colors[i].fadeToBlackBy(5);
                if(CRGB(dist, dist, dist) > ledMap->colors[i]){
                    ledMap->colors[i] = CRGB(dist, dist, dist);
                }
            }
        }
};
RadarSweepEffect::RadarSweepEffect(void) {
    angle = 0;
}

LEDMap leds;
TwinkleEffect twinkleEffect;
RadarSweepEffect radarSweepEffect;



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
    FastLED.clear();

}

void loop() {
//    twinkleEffect.update(&leds);
    radarSweepEffect.update(&leds);
    Serial.println(leds.colors[0][0]);
    FastLED.show();
    delay(1);
}
