#include <Wire.h>
#include <MPU6050.h>
#include <FastLED.h>
#include <math.h>
#include "PixelMap.h"
//#include <Bounce2.h>



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
128,   0,0,  255,
200,   255,0,  255,   //bright yellow
255,   255,255,255 }; //full white

CRGBPalette256 myPal = heatmap_gp;

//class BallsEffect {
//
//    public:
//
//    void update(LEDMap *ledMap, int16_t angle) {
//
//    }
//}

class RainbowCylinderEffect {

    public:
        void update(LEDMap *ledMap, int16_t angle) {
            for(int i = 0; i<NUM_LEDS; i++){
                uint16_t dist = ledMap->getPositiveThetaDistance(i, angle);
                ledMap->colors[i] = CHSV(dist * 255 / 360, 255, 255);
//                ledMap->colors[i].fadeToBlackBy(20);
//                if(CRGB(dist, dist, dist) > ledMap->colors[i]){
//                    ledMap->colors[i] = CRGB(dist, dist, dist);
//                }
            }
        }
};

class RadarSweepEffect {

    public:
        uint8_t LED_color_indices[NUM_LEDS] = {0};

        void update(LEDMap *ledMap, int16_t angle) {
            for(int i = 0; i<NUM_LEDS; i++){
                int16_t dist = ledMap->getThetaDistance(i, angle);
                dist = map(dist, 0, 30, 255, 0);
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
        void update(int16_t, uint32_t dt);
        int16_t velocity;
        float torque;
};

void Torque::update(int16_t new_vel, uint32_t dt){
    float new_torque = (new_vel - velocity) / (float)dt * 100000.0;
    velocity = new_vel;
//    torque = new_torque;
    torque += (((float)dt / 1000000) / 0.1) * (new_torque - torque);

//    Serial.print(torque);
//    Serial.print(" ");
//    Serial.println((int16_t)velocity);

}

class IMUTracker {
    public:
    IMUTracker();
    float theta_x, theta_y, theta_z;
    void update(int16_t, uint32_t);
};

IMUTracker::IMUTracker(void){
    theta_x = 0;
}

void IMUTracker::update(int16_t dtheta_x, uint32_t dt){
    int16_t div;
    theta_x -= dtheta_x  * 0.068702f * 0.150f *  (dt / 1000000.0); // for 200 degrees/s

    // Modulo 360 (for floating point)
    div = theta_x / 360.0;
    div = floor(div);
    theta_x = theta_x - (div*360);
    if(theta_x < 0){
        theta_x += 360;
    }
}

class Spinner {
    public:
    float MAX_VELOCITY = 200;
    Spinner();
    float angle;
    float velocity;
    void update(float, int16_t, uint32_t);

};

Spinner::Spinner(void){
    angle = 0;
    velocity = 0;
}
void Spinner::update(float torque, int16_t gyro_velocity, uint32_t dt){
    float div;
    if(torque * gyro_velocity > 0 && abs(gyro_velocity) > 400) { // If torque and velocity are in the same direction
        velocity += torque * (dt / 1000000.0) * 4; // Increase velocity by torque
    }
//    if(velocity > MAX_VELOCITY){
//        velocity = MAX_VELOCITY;
//    } else if (velocity < -MAX_VELOCITY) {
//        velocity = -MAX_VELOCITY;
//    }
    angle += velocity * (dt / 1000000.0) * 0.0687f;
    div = angle / 360.0;
    div = floor(div);
    angle = angle - (div*360);

    if(velocity > 0){
        velocity -= 2;
    } else {
        velocity += 2;
    }
}

LEDMap leds;
TwinkleEffect twinkleEffect;
RadarSweepEffect radarSweepEffect;
RainbowCylinderEffect rainbowCylinderEffect;
Torque torque;
Spinner spin;
MPU6050 mpu;
elapsedMicros dt;
IMUTracker imu_tracker;

uint8_t POWER_ENABLE_PIN = 0;
uint8_t BUTTON_PIN = 1;
uint8_t LED_pin = 13;

void setup() {

    pinMode(BUTTON_PIN, INPUT);
    pinMode(POWER_ENABLE_PIN, OUTPUT);
    pinMode(LED_pin, OUTPUT);
    digitalWrite(POWER_ENABLE_PIN, HIGH);
    digitalWrite(LED_pin, HIGH);

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

uint8_t STATE = 0;
uint8_t MODE = 0;
uint8_t MAX_MODE = 3;
uint16_t POWER_OFF_TIME = 2000;

void update_state(uint8_t button_press) {
    static elapsedMillis button_held;

    if( STATE == 0 ){
        // Just powered on, button will be pressed and then transition to released
        if(button_press == 0){
            STATE = 1; // Button is released, transition to running STATE
            Serial.println("Normal mode");
        }
    }
    else if( STATE == 1 ){
        // Normal state, button not pressed
        if (button_press == 1){
            button_held = 0; // Start timer for button press
            STATE = 2; // Transition to button pressed state
            Serial.println("Button pressed");
        }
    }
    else if( STATE == 2){
        // Button held down
        if (button_press == 0){
            MODE++;
            MODE %= MAX_MODE;
            STATE = 1; // Transition back to released state
            Serial.println("Button released");
            Serial.print("Mode: ");
            Serial.println(MODE);
        }
        Serial.print("Button held: ");
        Serial.println(button_held);
        if (button_held > POWER_OFF_TIME){
            Serial.println("Powering down...");
            digitalWrite(POWER_ENABLE_PIN, LOW); // Button held down for too long, power down
            STATE = 3; // Power down mode, shut down effects
            MODE = MAX_MODE;
        }
    }

}

void handle_mode(LEDMap *leds){
    static int16_t angle = 0;
    static uint8_t prev_mode = 0;
    static elapsedMillis t;
    float rawX;
    if (MODE != prev_mode) {
        leds->clearAll();
        prev_mode = MODE;
    }
    if (MODE == 0) {
        rainbowCylinderEffect.update(leds, angle);
        angle++;
        angle %= 360;
    }
    else if (MODE == 1) {
        Vector rawGyro = mpu.readRawGyro();
        torque.update(rawGyro.XAxis, dt);
        spin.update(torque.torque, rawGyro.XAxis, dt);
        radarSweepEffect.update(leds, spin.angle);
    }
    else if (MODE == 2) {
        Vector rawGyro = mpu.readRawGyro();
        imu_tracker.update(rawGyro.XAxis, dt);
        rainbowCylinderEffect.update(leds, imu_tracker.theta_x);
//        Serial.println(imu_tracker.theta_x);
        // Vector rawAccel = mpu.readRawAccel();
//        Serial.print(t);
//        Serial.print(" ");
//        Serial.print((int16_t)rawAccel.XAxis);
//        Serial.print(" ");
//        Serial.print((int16_t)rawAccel.YAxis);
//        Serial.print(" ");
//        Serial.print((int16_t)rawAccel.ZAxis);
//        Serial.print(" ");
//        Serial.print((int16_t)rawGyro.XAxis);
//        Serial.print(" ");
//        Serial.print((int16_t)rawGyro.YAxis);
//        Serial.print(" ");
//        Serial.println((int16_t)rawGyro.ZAxis);

    }
}

void loop() {

    update_state(digitalRead(BUTTON_PIN));
    handle_mode(&leds);
    FastLED.show();
    dt = 0;
    delay(10);
}