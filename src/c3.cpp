#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

// --- Config ---
#define LED_PIN 4
#define NUM_LEDS 5
#define BLINK_FAST_MS 150
#define ESTOP_TOGGLE_MS 90
#define FINISHED_TARGET_TOGGLE_MS 280
#define LIGHTHOUSE_SWEEP_MS 900

Adafruit_NeoPixel led(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

enum RobotState {
    NO_ROBOT,
    HELLO_ROBOT, // first connect
    DISABLE,
    ENABLE_TELEOP,
    ENABLE_AUTONOMOUS,
    FINISHED_TARGET, // used to show completion of goal, etc
    ESTOP,
};
volatile RobotState current_state = HELLO_ROBOT;

void setAll(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < NUM_LEDS; i++) {
        led.setPixelColor(i, led.Color(r, g, b));
    }
}

uint8_t log_breathe(unsigned long now, unsigned long period_ms) {
    float t = fmod((float)now, (float)period_ms) / (float)period_ms;
    float linear = (t < 0.5f) ? (t * 2.0f) : (2.0f - t * 2.0f);

    // Exponential: lingers dark, punches bright — looks natural on LEDs
    float curved = (expf(linear * 3.0f) - 1.0f) / (expf(3.0f) - 1.0f);

    return (uint8_t)(curved * 255.0f);
}

uint8_t log_breathe_floor(unsigned long now, unsigned long period_ms, uint8_t floor_brightness) {
    uint8_t breath = log_breathe(now, period_ms);
    uint16_t span = 255 - floor_brightness;
    return floor_brightness + (uint8_t)((span * breath) / 255);
}

void setRainbowBreathe(unsigned long now, uint8_t brightness, unsigned long cycle_ms) {
    uint16_t base_hue = (uint16_t)((now % cycle_ms) * 65535UL / cycle_ms);
    uint16_t hue_step = (uint16_t)(65535UL / NUM_LEDS);

    for (int i = 0; i < NUM_LEDS; i++) {
        uint16_t hue = base_hue + (uint16_t)(i * hue_step);
        led.setPixelColor(i, led.gamma32(led.ColorHSV(hue, 255, brightness)));
    }
}

void setLighthouseSweep(unsigned long now, uint8_t red, uint8_t green, uint8_t blue,
                        unsigned long sweep_ms) {
    float position = fmodf((float)now / (float)sweep_ms, (float)NUM_LEDS);

    for (int i = 0; i < NUM_LEDS; i++) {
        float distance = fabsf((float)i - position);
        distance = fminf(distance, (float)NUM_LEDS - distance);

        float intensity = 1.0f - (distance / 1.5f);
        if (intensity < 0.0f) {
            intensity = 0.0f;
        }

        float shaped = intensity * intensity;
        led.setPixelColor(i, led.Color((uint8_t)(red * shaped), (uint8_t)(green * shaped),
                                       (uint8_t)(blue * shaped)));
    }
}

void setEstopToggle(unsigned long now) {
    bool left_side = ((now / ESTOP_TOGGLE_MS) % 2) == 0;

    led.clear();
    if (left_side) {
        led.setPixelColor(0, led.Color(255, 0, 0));
        led.setPixelColor(1, led.Color(255, 0, 0));
    } else {
        led.setPixelColor(3, led.Color(255, 0, 0));
        led.setPixelColor(4, led.Color(255, 0, 0));
    }
}

void setHappyTargetToggle(unsigned long now) {
    bool left_side = ((now / FINISHED_TARGET_TOGGLE_MS) % 2) == 0;

    led.clear();
    if (left_side) {
        led.setPixelColor(1, led.Color(0, 255, 0));
    } else {
        led.setPixelColor(3, led.Color(0, 255, 0));
    }
}

// --- LED driver ---
void update_led() {
    static bool blink_on = false;
    static unsigned long last_blink = 0;
    unsigned long now = millis();

    switch (current_state) {
        case NO_ROBOT: {
            setLighthouseSweep(now, 50, 50, 50, LIGHTHOUSE_SWEEP_MS);
            led.show();
            break;
        }

        case HELLO_ROBOT: {
            uint8_t brightness = log_breathe_floor(now, 800, 100);
            setRainbowBreathe(now, brightness, 1200);
            led.show();
            break;
        }

        case DISABLE: {
            float brightness = log_breathe_floor(now, 4000, 10);
            setAll(brightness / 10.0, brightness / 10.0, brightness);
            led.show();
            break;
        }

        case ENABLE_TELEOP: {
            if (now - last_blink >= BLINK_FAST_MS) {
                blink_on = !blink_on;
                last_blink = now;
            }
            setAll(blink_on ? 255 : 0, blink_on ? 30 : 0, 0);
            led.show();
            break;
        }

        case ENABLE_AUTONOMOUS: {
            // if (now - last_blink >= BLINK_FAST_MS) {
            //     blink_on = !blink_on;
            //     last_blink = now;
            // }
            // setAll(blink_on ? 180 : 0, 0, blink_on ? 120 : 0);
            setLighthouseSweep(now, 255, 0, 180, 80);
            led.show();
            break;
        }

        case FINISHED_TARGET: {
            setHappyTargetToggle(now);
            led.show();
            break;
        }

        case ESTOP: {
            setEstopToggle(now);
            led.show();
            break;
        }
    }
}

uint32_t last_cmd_received = 0;
uint32_t hello_time {0};
static constexpr uint32_t CMD_TIMEOUT_MS = 4000;
static constexpr uint32_t HELLO_TIMEOUT_MS = 3000;

void setup() {
    led.begin();
    led.setBrightness(255);
    led.clear();
    led.show();

    Serial.begin(115200);
}

void loop() {
    if (Serial.available() > 0) {
        static RobotState newState = NO_ROBOT;
        char cmd = Serial.read();
        last_cmd_received = millis();
        switch (cmd) {
            case 'n':
                newState = NO_ROBOT;
                break;
            case 'h':
                hello_time = millis();
                newState = HELLO_ROBOT;
                break;
            case 'd':
                newState = DISABLE;
                break;
            case 'e':
                newState = ENABLE_TELEOP;
                break;
            case 'a':
                newState = ENABLE_AUTONOMOUS;
                break;
            case 'f':
                newState = FINISHED_TARGET;
                break;
            case 's':
                newState = ESTOP;
                break;
        }

        // If we are disabled and have just connected, show a greeting state
        // for HELLO_TIMEOUT_MS
        if (current_state == HELLO_ROBOT && newState == DISABLE &&
            millis() - hello_time < HELLO_TIMEOUT_MS) {
            update_led();
            return;
        } else if (current_state == HELLO_ROBOT && millis() - hello_time >= HELLO_TIMEOUT_MS) {
            hello_time = 0;
            newState = DISABLE;
        }

        current_state = newState;

    } else if (current_state != NO_ROBOT && millis() - last_cmd_received > CMD_TIMEOUT_MS) {
        current_state = NO_ROBOT;
    }

    update_led();
}
