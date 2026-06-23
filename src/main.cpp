#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error "This program requires Arduino serial micro-ROS transport."
#endif

constexpr uint8_t TRIG_PIN = 5;
constexpr uint8_t ECHO_PIN = 18;
constexpr unsigned long ECHO_TIMEOUT_US = 30000UL;

constexpr uint32_t WAITING_PING_INTERVAL_MS = 500;
constexpr uint32_t CONNECTED_PING_INTERVAL_MS = 500;

// ROS 2 entities
rcl_node_t node;
rcl_publisher_t publisher;
rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

std_msgs__msg__Float32 distance_msg;

enum ConnectionState
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

ConnectionState connection_state = WAITING_AGENT;
uint32_t last_ping_ms = 0;

#define RCCHECK(function_call)                    \
    do                                            \
    {                                             \
        rcl_ret_t return_code = function_call;    \
        if (return_code != RCL_RET_OK)            \
        {                                         \
            return false;                         \
        }                                         \
    } while (0)

#define RCSOFTCHECK(function_call)                 \
    do                                            \
    {                                             \
        rcl_ret_t return_code = function_call;    \
        (void)return_code;                        \
    } while (0)

bool interval_elapsed(uint32_t interval_ms)
{
    const uint32_t now = millis();

    if ((uint32_t)(now - last_ping_ms) >= interval_ms)
    {
        last_ping_ms = now;
        return true;
    }

    return false;
}

float measure_distance_cm()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    const unsigned long duration_us =
        pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);

    if (duration_us == 0)
    {
        return -1.0f;
    }

    return duration_us * 0.0343f / 2.0f;
}float measure_filtered_distance_cm()
{
    constexpr uint8_t SAMPLE_COUNT = 7;
    float samples[SAMPLE_COUNT];
    uint8_t valid_count = 0;

    for (uint8_t i = 0; i < SAMPLE_COUNT; i++)
    {
        const float measurement = measure_distance_cm();

        if (measurement >= 0.0f)
        {
            samples[valid_count] = measurement;
            valid_count++;
        }

        // Allow previous ultrasonic echoes to dissipate.
        delay(60);
    }

    // No valid echo was received.
    if (valid_count == 0)
    {
        return -1.0f;
    }

    // Sort valid samples from smallest to largest.
    for (uint8_t i = 0; i < valid_count - 1; i++)
    {
        for (uint8_t j = i + 1; j < valid_count; j++)
        {
            if (samples[j] < samples[i])
            {
                const float temporary = samples[i];
                samples[i] = samples[j];
                samples[j] = temporary;
            }
        }
    }

    // Return the middle value.
    return samples[valid_count / 2];
}

void timer_callback(rcl_timer_t *timer_handle, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer_handle != nullptr)
    {
        distance_msg.data = measure_filtered_distance_cm();

        RCSOFTCHECK(rcl_publish(
            &publisher,
            &distance_msg,
            nullptr
        ));
    }
}


bool create_entities()
{
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(
        &support,
        0,
        nullptr,
        &allocator
    ));

    RCCHECK(rclc_node_init_default(
        &node,
        "esp32_sensor_bridge",
        "",
        &support
    ));

    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "ultrasonic_distance_cm"
    ));

    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback
    ));

    executor = rclc_executor_get_zero_initialized_executor();

    RCCHECK(rclc_executor_init(
        &executor,
        &support.context,
        1,
        &allocator
    ));

    RCCHECK(rclc_executor_add_timer(
        &executor,
        &timer
    ));

    distance_msg.data = -1.0f;

    return true;
}

void destroy_entities()
{
    rmw_context_t *rmw_context =
        rcl_context_get_rmw_context(&support.context);

    if (rmw_context != nullptr)
    {
        // Do not wait for replies from an Agent that has disconnected.
        rmw_uros_set_context_entity_destroy_session_timeout(
            rmw_context,
            0
        );
    }

    RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
    RCSOFTCHECK(rcl_timer_fini(&timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}

void setup()
{
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);

    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    connection_state = WAITING_AGENT;
    last_ping_ms = 0;
}

void loop()
{
    switch (connection_state)
    {
        case WAITING_AGENT:
            if (interval_elapsed(WAITING_PING_INTERVAL_MS))
            {
                connection_state =
                    (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
                    ? AGENT_AVAILABLE
                    : WAITING_AGENT;
            }
            break;

        case AGENT_AVAILABLE:
            connection_state =
                create_entities()
                ? AGENT_CONNECTED
                : WAITING_AGENT;

            if (connection_state == WAITING_AGENT)
            {
                destroy_entities();
            }

            last_ping_ms = millis();
            break;

        case AGENT_CONNECTED:
            if (interval_elapsed(CONNECTED_PING_INTERVAL_MS))
            {
                if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
                {
                    connection_state = AGENT_DISCONNECTED;
                }
            }

            if (connection_state == AGENT_CONNECTED)
            {
                RCSOFTCHECK(rclc_executor_spin_some(
                    &executor,
                    RCL_MS_TO_NS(20)
                ));
            }
            break;

        case AGENT_DISCONNECTED:
            destroy_entities();
            connection_state = WAITING_AGENT;
            last_ping_ms = millis();
            break;
    }

    delay(10);
}

