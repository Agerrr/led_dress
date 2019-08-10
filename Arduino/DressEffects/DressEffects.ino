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

LEDMap leds;

void setup() {

    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds.colors, NUM_LEDS);
    leds.updateLEDColor(5, CRGB(255,255,255));
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

//    FastLED.clear();
    FastLED.show();
}
