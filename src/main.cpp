#include <Adafruit_NeoPixel.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/bool.h>

// --- Config ---
#define LED_PIN 48
#define BLINK_FAST_MS 150

#define TOPIC_CONTROL "/pidge/tandem_robot_controller/control_state"
#define TOPIC_ESTOP "/pidge/tandem_robot_controller/estop_state"

Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);

enum RobotState {
    NO_ROBOT,
    DISABLE,
    ENABLE,
    ESTOP,
};
volatile RobotState current_state = NO_ROBOT;

// micro-ROS
rcl_subscription_t control_sub;
rcl_subscription_t estop_sub;
std_msgs__msg__Bool control_msg;
std_msgs__msg__Bool estop_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

enum AgentState { WAITING_AGENT, AGENT_CONNECTED };
AgentState agent_state = WAITING_AGENT;

// --- Callbacks ---
void control_callback(const void* msgin) {
    const std_msgs__msg__Bool* m = (const std_msgs__msg__Bool*)msgin;
    if (current_state == ESTOP) {
        return;
    }
    current_state = m->data ? ENABLE : DISABLE;
}

void estop_callback(const void* msgin) {
    const std_msgs__msg__Bool* m = (const std_msgs__msg__Bool*)msgin;
    if (m->data) {
        current_state = ESTOP;
    }
}

uint8_t log_breathe(unsigned long now, unsigned long period_ms) {
    float t = fmod((float)now, (float)period_ms) / (float)period_ms;
    float linear = (t < 0.5f) ? (t * 2.0f) : (2.0f - t * 2.0f);

    // Exponential: lingers dark, punches bright — looks natural on LEDs
    // Replace the old log1p line with this:
    float curved = (expf(linear * 3.0f) - 1.0f) / (expf(3.0f) - 1.0f);

    return (uint8_t)(curved * 255.0f);
}

// --- LED driver ---
void update_led() {
    static bool blink_on = false;
    static unsigned long last_blink = 0;
    unsigned long now = millis();

    // bool robot_connected = (rmw_uros_ping_agent(50, 1) == RMW_RET_OK);

    // if (!robot_connected) {
    //     current_state = NO_ROBOT;
    // }

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

void create_entities() {
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "rsl_node", "", &support);

    rclc_subscription_init_default(&control_sub, &node,
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TOPIC_CONTROL);
    rclc_subscription_init_default(&estop_sub, &node,
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TOPIC_ESTOP);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &control_sub, &control_msg, &control_callback,
                                   ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &estop_sub, &estop_msg, &estop_callback, ON_NEW_DATA);
}

void destroy_entities() {
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_subscription_fini(&control_sub, &node);
    rcl_subscription_fini(&estop_sub, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

void setup() {
    led.begin();
    led.setBrightness(255);
    led.clear();
    led.show();

    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(1000);

    // allocator = rcl_get_default_allocator();
    // rclc_support_init(&support, 0, NULL, &allocator);
    // rclc_node_init_default(&node, "rsl_node", "", &support);

    // rclc_subscription_init_default(&control_sub, &node,
    //                                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    //                                TOPIC_CONTROL);
    // rclc_subscription_init_default(&estop_sub, &node,
    //                                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    //                                TOPIC_ESTOP);

    // rclc_executor_init(&executor, &support.context, 2, &allocator);
    // rclc_executor_add_subscription(&executor, &control_sub, &control_msg, &control_callback,
    //                                ON_NEW_DATA);
    // rclc_executor_add_subscription(&executor, &estop_sub, &estop_msg, &estop_callback,
    // ON_NEW_DATA);
}

void loop() {
    switch (agent_state) {
        case WAITING_AGENT:
            set_microros_serial_transports(Serial);
            if (rmw_uros_ping_agent(500, 1) == RMW_RET_OK) {
                create_entities();
                current_state = NO_ROBOT;
                agent_state = AGENT_CONNECTED;
                delay(500);
            } else {
                delay(20);
            }
            break;

        case AGENT_CONNECTED:
            if (rmw_uros_ping_agent(4000, 1) == RMW_RET_OK) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            } else {
                destroy_entities();

                current_state = NO_ROBOT;
                agent_state = WAITING_AGENT;
            }
            break;
    }

    update_led();
}
