#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

// --- Config ---
#define LED_PIN 48
#define BLINK_FAST_MS 150

#define TOPIC_CONTROL "/lance/tandem_robot_controller/control_state"
#define TOPIC_ESTOP "/lance/tandem_robot_controller/estop_state"

Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);

enum RobotState {
    NO_ROBOT,
    DISABLE,
    ENABLE,
    ESTOP,
};
volatile RobotState current_state = NO_ROBOT;

uint8_t log_breathe(unsigned long now, unsigned long period_ms) {
    float t = fmod((float)now, (float)period_ms) / (float)period_ms;
    float linear = (t < 0.5f) ? (t * 2.0f) : (2.0f - t * 2.0f);

    // Exponential: lingers dark, punches bright — looks natural on LEDs
    float curved = (expf(linear * 3.0f) - 1.0f) / (expf(3.0f) - 1.0f);

    return (uint8_t)(curved * 255.0f);
}

// --- LED driver ---
void update_led() {
    static bool blink_on = false;
    static unsigned long last_blink = 0;
    unsigned long now = millis();

    switch (current_state) {
        case NO_ROBOT: {
            led.setPixelColor(0, 0);
            led.show();
            break;
        }

        case DISABLE: {
            uint8_t brightness = log_breathe(now, 2500);
            led.setPixelColor(0, led.Color(0, 0, brightness));
            led.show();
            break;
        }

        case ENABLE: {
            if (now - last_blink >= BLINK_FAST_MS) {
                blink_on = !blink_on;
                last_blink = now;
            }
            led.setPixelColor(0, blink_on ? led.Color(255, 50, 0) : 0);
            led.show();
            break;
        }

        case ESTOP: {
            led.setPixelColor(0, led.Color(255, 0, 0));
            led.show();
            break;
        }
    }
}

uint32_t last_cmd_received = 0;
static constexpr uint32_t CMD_TIMEOUT_MS = 2000;

void setup() {
    led.begin();
    led.setBrightness(255);
    led.clear();
    led.show();

    Serial.begin(115200);
    
}

void loop() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        last_cmd_received = millis();
        switch (cmd) {
            case 'n':
                current_state = NO_ROBOT;
                break;
            case 'd':
                current_state = DISABLE;
                break;
            case 'e':
                current_state = ENABLE;
                break;
            case 's':
                current_state = ESTOP;
                update_led();
                sleep(CMD_TIMEOUT_MS);
                current_state = NO_ROBOT;
                break;
        }
    } else if (current_state != NO_ROBOT && millis() - last_cmd_received > CMD_TIMEOUT_MS) {
        current_state = NO_ROBOT;
    }

    update_led();
}
