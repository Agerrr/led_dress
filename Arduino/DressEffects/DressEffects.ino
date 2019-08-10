#include <Wire.h>
#include <MPU6050.h>
#include <FastLED.h>

#define NUM_LEDS 60
#define DATA_PIN 17

MPU6050 mpu;

struct coordinate {
  int16_t theta;    // Angles [0, 360]
  int16_t z;        // Vertical axis [0, 100]
};

class LEDMap {

    public:
        coordinate cords[NUM_LEDS];
        CRGB colors[NUM_LEDS];

        void clear();

        void updateLEDColor(uint16_t led_index, CRGB color) {
            colors[led_index] = color;
        }

        void setCoordinates(coordinate cords[]);
        uint16_t getDistance(coordinate point_cord, uint16_t led_index);
};

class TwinkleEffect {

    public:

        void update(LEDMap *ledMap) {
            for(int i = 0; i < NUM_LEDS; i++) {
                ledMap.colors[i] = CRGB(255, 0, 0);
                if(ledMap.colors[i] == CRGB(0, 0, 0)) {
                    ledMap.colors[i] = CRGB(255, 255, 255);
                } else {
                    ledMap.colors[i] -= CRGB(1, 1, 1);
                }
            }
        }
};

LEDMap leds;
TwinkleEffect twinkleEffect;

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
}

void loop() {

    twinkleEffect.update(&leds);
    Serial.println(leds.colors[0][0]);
    FastLED.show();
}
