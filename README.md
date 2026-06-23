# ESP32 ↔ ROS 2 Sensor Bridge



An embedded sensor interface that reads distance measurements using an ESP32 and publishes them as ROS 2 topics through micro-ROS over serial transport.

## Project Overview

The ESP32 reads data from an HC-SR04 ultrasonic sensor and publishes the measured distance to a ROS 2 topic. A micro-ROS agent runs on the laptop and acts as the communication bridge between the ESP32 and the ROS 2 network.

```text
HC-SR04 Sensor
      ↓
    ESP32  (micro-ROS node)
      ↓  USB / Serial
micro-ROS Agent  (laptop)
      ↓
 ROS 2 Topic: /ultrasonic_distance_cm
```

## Features

- Reads distance measurements from an HC-SR04 ultrasonic sensor
- Publishes sensor data as a ROS 2 `std_msgs/msg/Float32` topic
- Uses micro-ROS on the ESP32 with serial transport
- Built with PlatformIO and the Arduino framework
- Includes connection recovery and sensor-data filtering
- Monitorable with standard ROS 2 command-line tools

## Hardware

- ESP32 development board (30-pin DevKit, CP2102 USB-serial)
- HC-SR04 ultrasonic sensor
- 1 kΩ and 2 kΩ resistors (voltage divider for Echo pin)
- Jumper wires and breadboard
- USB cable

## Software

- Ubuntu 22.04
- ROS 2 Humble
- micro-ROS (micro_ros_platformio library)
- micro-ROS Agent (built from source)
- PlatformIO
- Arduino framework / C++

## Wiring

| HC-SR04 Pin | ESP32 Connection                          |
|-------------|-------------------------------------------|
| VCC         | VIN (5 V)                                 |
| GND         | GND                                       |
| TRIG        | GPIO 5                                    |
| ECHO        | GPIO 18 via voltage divider (1 kΩ / 2 kΩ)|

> The HC-SR04 Echo output is 5 V. The ESP32 GPIO operates at 3.3 V.  
> A voltage divider (1 kΩ in series, 2 kΩ to GND) reduces the signal to ~3.3 V.

## Repository Structure

```text
esp32_ros2_bridge/
├── platformio.ini
├── src/
│   └── main.cpp
└── .gitignore
```

- `platformio.ini` — PlatformIO board, framework, library, and micro-ROS configuration
- `src/main.cpp` — ESP32 firmware (sensor reading + micro-ROS publisher)
- `.gitignore` — excludes build artifacts and backup files

## platformio.ini

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino

board_microros_distro = humble
board_microros_transport = serial

lib_deps =
    https://github.com/micro-ROS/micro_ros_platformio
```

## Build and Upload

```bash
# Navigate to project
cd ~/esp32_ros2_bridge

# Build firmware
pio run

# Upload to ESP32
pio run --target upload

# Open serial monitor (optional debug)
pio device monitor
```

## Start the micro-ROS Agent

```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyUSB0 \
  -b 115200
```

> The serial port may also appear as `/dev/ttyACM0` depending on your system.

## View the ROS 2 Topic

```bash
# List active topics
ros2 topic list

# Stream distance data continuously
ros2 topic echo /ultrasonic_distance_cm

# Print one message
ros2 topic echo /ultrasonic_distance_cm --once

# Check message type
ros2 topic type /ultrasonic_distance_cm
```

## Example Output

```text
data: 23.4
---
data: 23.1
---
data: 41.8
```

The value is the measured distance in centimetres as a `Float32`.

## How It Works

1. The ESP32 sends a 10 µs trigger pulse to the HC-SR04 TRIG pin (GPIO 5)
2. The sensor emits an ultrasonic burst and raises the ECHO pin (GPIO 18) high
3. The ESP32 measures the ECHO pulse duration using `pulseIn()`
4. Distance is calculated: `distance_cm = duration_µs × 0.0343 / 2`
5. Readings are filtered to reduce noise
6. micro-ROS publishes the value as a `std_msgs/msg/Float32` message
7. The micro-ROS Agent forwards it into the ROS 2 network
8. The topic is readable on the laptop with `ros2 topic echo`

## Demo




## Possible Improvements

- Migrate to `sensor_msgs/msg/Range` with proper ROS frame ID and timestamp
- Add multiple sensor support (front, left, right)
- Visualize distance in RViz using marker messages
- Switch to Wi-Fi (UDP) transport for wireless operation
- Add configurable sensor parameters (max range, field of view)
- Package the laptop-side configuration as a proper ROS 2 package

## Author

**Shikhar Yadav**  
GitHub: [shikharyadav-in](https://github.com/shikharyadav-in)  
LinkedIn: [shikhar-yadav-300540250](https://linkedin.com/in/shikhar-yadav-300540250)
