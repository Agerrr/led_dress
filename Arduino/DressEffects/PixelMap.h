#include <FastLED.h>
#include <stdlib.h>

#define NUM_LEDS 80
#define DATA_PIN 17

class LEDMap {

    public:
        int16_t cords[NUM_LEDS][2] = {
            {0, 0},
            {42, 0},
            {85, 0},
            {127, 0},
            {169, 0},
            {212, 0},
            {254, 0},
            {296, 0},
            {339, 0},
            {10, 60},
            {46, 60},
            {82, 60},
            {118, 60},
            {154, 60},
            {190, 60},
            {226, 60},
            {262, 60},
            {298, 60},
            {334, 60},
            {10, 120},
            {75, 120},
            {108, 120},
            {141, 120},
            {174, 120},
            {206, 120},
            {239, 120},
            {272, 120},
            {305, 120},
            {337, 120},
            {0, 180},
            {27, 180},
            {53, 180},
            {80, 180},
            {107, 180},
            {133, 180},
            {160, 180},
            {187, 180},
            {213, 180},
            {240, 180},
            {267, 180},
            {320, 180},
            {347, 180},
            {10, 240},
            {35, 240},
            {60, 240},
            {84, 240},
            {109, 240},
            {134, 240},
            {159, 240},
            {184, 240},
            {209, 240},
            {233, 240},
            {258, 240},
            {283, 240},
            {308, 240},
            {333, 240},
            {358, 240},
            {10, 300},
            {30, 300},
            {50, 300},
            {90, 300},
            {110, 300},
            {130, 300},
            {150, 300},
            {170, 300},
            {190, 300},
            {210, 300},
            {230, 300},
            {250, 300},
            {270, 300},
            {290, 300},
            {310, 300},
            {330, 300},
            {350, 300},
            {10, 360},
            {30, 360},
            {50, 360},
            {70, 360},
            {90, 360},
            {110, 360}
        };
        CRGB colors[NUM_LEDS];

        void updateLEDColor(uint16_t led_index, CRGB color) {
            colors[led_index] = color;
        }

        void clearAll() {
            for(int i = 0; i < NUM_LEDS; i++) {
                colors[i] = CRGB(0,0,0);
            }
        }

        void setCoordinates(int16_t cords[]);

        uint16_t getThetaDistance(uint16_t point_index, int16_t theta) {
            int16_t diff = abs(theta - cords[point_index][0]);
            return min(diff, 360-diff);
        }

        uint16_t getDistance(uint16_t point_index, int16_t theta, int16_t height) {
            uint16_t dtheta = getThetaDistance(point_index, theta);
            uint16_t dh = abs(cords[point_index][1] - height);
            return dh + dtheta;
        }


};

