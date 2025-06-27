# MadFlight IBus Library

A modern, event-driven C++ library for handling Flysky/Turnigy RC iBUS protocol, specifically designed for MadFlight boards using Mf_Serial.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Features

- **Event-Driven Architecture**: Uses callback-based processing for real-time responsiveness
- **Modern C++ Design**: Leverages std::function for flexible callback management
- **Dual Processing Modes**: Both event-driven callbacks and traditional polling approaches
- **Comprehensive Statistics**: Built-in communication monitoring and error tracking
- **Flexible Sensor Management**: Dynamic sensor registration and data updates
- **MadFlight Optimized**: Designed specifically for MadFlight boards with Mf_Serial

## Installation

1. Download the library files
2. Place in your Arduino libraries folder
3. Include the library in your sketch: `#include <IBus.h>`

## Quick Start

### Basic Setup

```cpp
#include <IBus.h>
#include <Mf_Serial.h>

// Define pins for Arduino Mega Serial1
#define IBUS_RX_PIN 19    // RX1 pin for iBUS receiver
#define IBUS_TX_PIN 18    // TX1 pin for iBUS (used for sensor telemetry)

Mf_Serial ibus_serial;
IBus ibus;

void setup() {
    // Initialize IBus with event-driven processing and specific pins
    ibus.begin(ibus_serial, IBus::NO_TIMER, IBUS_RX_PIN, IBUS_TX_PIN);

    // Set up callbacks for real-time processing
    ibus.on_channel_update([](uint8_t channel, uint16_t value) {
        Serial.print("Channel ");
        Serial.print(channel);
        Serial.print(": ");
        Serial.println(value);
    });

    ibus.on_sensor_request([](uint8_t sensor_id, uint8_t request_type) {
        // Handle sensor requests here
        Serial.print("Sensor ");
        Serial.print(sensor_id);
        Serial.print(" requested: ");
        Serial.println(request_type);
    });
}

void loop() {
    // Process events (call this regularly)
    ibus.process_events();
}
```

### Adding Sensors

```cpp
void setup() {
    // Initialize with specific pins
    ibus.begin(ibus_serial, IBus::NO_TIMER, IBUS_RX_PIN, IBUS_TX_PIN);

    // Register sensors for telemetry
    ibus.register_sensor(1, IBus::SENSOR_INTERNAL_VOLTAGE, 2);
    ibus.register_sensor(2, IBus::SENSOR_TEMPERATURE, 2);
    ibus.register_sensor(3, IBus::SENSOR_RPM, 4);
}

void loop() {
    ibus.process_events();

    // Update sensor data
    ibus.update_sensor_data(1, 1250);  // 12.50V
    ibus.update_sensor_data(2, 250);   // 25.0°C
    ibus.update_sensor_data(3, 8500);  // 8500 RPM
}
```

## Pin Configuration

### Arduino Mega Pin Setup

The library uses `Mf_Serial` which requires specific pin configuration. For Arduino Mega, we use the built-in hardware serial ports:

```cpp
// Define pins for Arduino Mega Serial1
#define IBUS_RX_PIN 19    // RX1 pin for iBUS receiver
#define IBUS_TX_PIN 18    // TX1 pin for iBUS (used for sensor telemetry)

// Initialize with pin configuration
ibus.begin(ibus_serial, IBus::NO_TIMER, IBUS_RX_PIN, IBUS_TX_PIN);
```

### Arduino Mega Serial Ports

| Serial Port | RX Pin | TX Pin | Usage                          |
| ----------- | ------ | ------ | ------------------------------ |
| Serial1     | 19     | 18     | iBUS receiver/sensor           |
| Serial2     | 17     | 16     | Additional iBUS (multimonitor) |
| Serial3     | 15     | 14     | Additional serial port         |

### Multiple iBUS Connections

For receivers with separate servo and sensor pins (like TGY-IA6B):

```cpp
// Servo connection (receive only) - Serial1
#define SERVO_RX_PIN 19    // RX1 pin for servo iBUS
#define SERVO_TX_PIN -1    // TX1 pin not used (receive only)

// Sensor connection (bidirectional) - Serial2
#define SENSOR_RX_PIN 17   // RX2 pin for sensor iBUS
#define SENSOR_TX_PIN 16   // TX2 pin for sensor iBUS (used for telemetry)

Mf_Serial ibus_servo_serial;
Mf_Serial ibus_sensor_serial;
IBus ibus_servo;
IBus ibus_sensor;

void setup() {
    ibus_servo.begin(ibus_servo_serial, IBus::NO_TIMER, SERVO_RX_PIN, SERVO_TX_PIN);
    ibus_sensor.begin(ibus_sensor_serial, IBus::NO_TIMER, SENSOR_RX_PIN, SENSOR_TX_PIN);
}
```

### Hardware Connections

#### Single iBUS Pin (e.g., FS-iA8X)

```
Receiver iBUS Pin ──┬── RX1 (pin 19)
                    │
                    └── TX1 (pin 18) via 1.2kΩ resistor or 1N4148 diode
```

#### Dual iBUS Pins (e.g., TGY-IA6B)

```
Receiver Servo Pin ── RX1 (pin 19)
Receiver Sensor Pin ──┬── RX2 (pin 17)
                      └── TX2 (pin 16) via diode for half-duplex
```

### Other Arduino Boards

For other Arduino boards, adjust the pin definitions:

#### Arduino Uno/Nano

```cpp
// Use SoftwareSerial or hardware Serial (disconnect during programming)
#define IBUS_RX_PIN 10    // SoftwareSerial RX
#define IBUS_TX_PIN 11    // SoftwareSerial TX
```

#### ESP32

```cpp
// Use any available GPIO pins
#define IBUS_RX_PIN 16    // GPIO16
#define IBUS_TX_PIN 17    // GPIO17
```

#### STM32 Blue Pill

```cpp
// Use hardware serial pins
#define IBUS_RX_PIN PA10  // USART1_RX
#define IBUS_TX_PIN PA9   // USART1_TX
```

## API Reference

### Initialization

#### `begin(Mf_Serial& serial, int8_t timer_id = 0, int8_t rx_pin = -1, int8_t tx_pin = -1)`

Initialize the IBus communication with event-driven processing.

- `serial`: Reference to Mf_Serial port
- `timer_id`: Timer ID for automatic processing (use `IBus::NO_TIMER` to disable)
- `rx_pin`: RX pin number (optional, auto-detected on MadFlight)
- `tx_pin`: TX pin number (optional, auto-detected on MadFlight)

### Event-Driven Callbacks

#### `on_channel_update(ChannelUpdateCallback callback)`

Set callback for channel updates. Called whenever channel values change.

```cpp
ibus.on_channel_update([](uint8_t channel, uint16_t value) {
    // Handle channel update
    Serial.printf("Channel %d: %d\n", channel, value);
});
```

#### `on_sensor_request(SensorRequestCallback callback)`

Set callback for sensor requests. Called when the receiver requests sensor data.

```cpp
ibus.on_sensor_request([](uint8_t sensor_id, uint8_t request_type) {
    // Handle sensor request
    switch(request_type) {
        case IBus::IBUS_CMD_SENSOR_DISCOVER:
            // Discovery request
            break;
        case IBus::IBUS_CMD_SENSOR_TYPE:
            // Type request
            break;
        case IBus::IBUS_CMD_SENSOR_DATA:
            // Data request
            break;
    }
});
```

#### `on_error(ErrorCallback callback)`

Set callback for error conditions.

```cpp
ibus.on_error([](uint8_t error_code) {
    switch(error_code) {
        case 1: Serial.println("Checksum error"); break;
        case 2: Serial.println("Protocol error"); break;
    }
});
```

### Polling Methods

#### `uint16_t get_channel_value(uint8_t channel)`

Get current channel value (polling approach).

- `channel`: Channel number (0-13)
- Returns: Channel value (1000-2000) or 0 if invalid

#### `bool has_new_data()`

Check if new data is available since last check.

- Returns: `true` if new data received

### Sensor Management

#### `bool register_sensor(uint8_t sensor_id, uint8_t sensor_type, uint8_t data_size = 2)`

Register a sensor for telemetry.

- `sensor_id`: Sensor ID (1-10)
- `sensor_type`: Sensor type (use `SENSOR_*` constants)
- `data_size`: Data size in bytes (2 or 4)
- Returns: `true` if registration successful

#### `bool update_sensor_data(uint8_t sensor_id, int32_t data)`

Update sensor data value.

- `sensor_id`: Sensor ID (1-10)
- `data`: Sensor data value
- Returns: `true` if update successful

### Processing

#### `void process_events()`

Process incoming data using event-driven approach. Call this regularly in your main loop.

### Statistics and Monitoring

#### `Statistics get_statistics()`

Get communication statistics.

```cpp
IBus::Statistics stats = ibus.get_statistics();
Serial.printf("Messages: %lu, Errors: %lu\n",
    stats.messages_processed, stats.checksum_errors);
```

#### `void reset_statistics()`

Reset all statistics counters.

#### `uint8_t get_active_sensor_count()`

Get the number of registered sensors.

## Constants

### Sensor Types

- `SENSOR_INTERNAL_VOLTAGE`: Internal voltage (0.01V)
- `SENSOR_TEMPERATURE`: Temperature (0.1°C, 0 = -40°C)
- `SENSOR_RPM`: RPM
- `SENSOR_EXTERNAL_VOLTAGE`: External voltage (0.01V)
- `SENSOR_PRESSURE`: Pressure (Pa)
- `SENSOR_SERVO`: Servo value

### Configuration

- `NO_TIMER`: Disable automatic timer processing
- `MAX_CHANNELS`: Maximum servo channels (14)
- `MAX_SENSORS`: Maximum sensors (10)
- `CHANNEL_MIN`: Minimum channel value (1000)
- `CHANNEL_MAX`: Maximum channel value (2000)
- `CHANNEL_CENTER`: Center/neutral value (1500)

## Examples

### Basic Channel Reading

```cpp
#include <IBus.h>
#include <Mf_Serial.h>

// Define pins for Arduino Mega Serial1
#define IBUS_RX_PIN 19    // RX1 pin for iBUS receiver
#define IBUS_TX_PIN -1    // TX1 pin not used (receive only)

Mf_Serial ibus_serial;
IBus ibus;

void setup() {
    Serial.begin(115200);
    ibus.begin(ibus_serial, IBus::NO_TIMER, IBUS_RX_PIN, IBUS_TX_PIN);
}

void loop() {
    ibus.process_events();

    // Read channel values
    uint16_t throttle = ibus.get_channel_value(0);
    uint16_t aileron = ibus.get_channel_value(1);
    uint16_t elevator = ibus.get_channel_value(2);
    uint16_t rudder = ibus.get_channel_value(3);

    if (ibus.has_new_data()) {
        Serial.printf("T:%d A:%d E:%d R:%d\n",
            throttle, aileron, elevator, rudder);
    }
}
```

### Advanced Event-Driven Example

```cpp
#include <IBus.h>
#include <Mf_Serial.h>

// Define pins for Arduino Mega Serial1
#define IBUS_RX_PIN 19    // RX1 pin for iBUS receiver
#define IBUS_TX_PIN 18    // TX1 pin for iBUS (used for sensor telemetry)

Mf_Serial ibus_serial;
IBus ibus;

// Global variables for sensor data
int32_t voltage = 0;
int32_t temperature = 0;
int32_t rpm = 0;

void setup() {
    Serial.begin(115200);
    ibus.begin(ibus_serial, IBus::NO_TIMER, IBUS_RX_PIN, IBUS_TX_PIN);

    // Set up event handlers
    ibus.on_channel_update(handle_channel_update);
    ibus.on_sensor_request(handle_sensor_request);
    ibus.on_error(handle_error);

    // Register sensors
    ibus.register_sensor(1, IBus::SENSOR_INTERNAL_VOLTAGE);
    ibus.register_sensor(2, IBus::SENSOR_TEMPERATURE);
    ibus.register_sensor(3, IBus::SENSOR_RPM, 4);
}

void loop() {
    ibus.process_events();

    // Update sensor data periodically
    static uint32_t last_update = 0;
    if (millis() - last_update > 100) {
        voltage = analogRead(A0) * 5.0 / 1024.0 * 100; // Convert to 0.01V
        temperature = read_temperature() * 10; // Convert to 0.1°C
        rpm = read_rpm();

        ibus.update_sensor_data(1, voltage);
        ibus.update_sensor_data(2, temperature);
        ibus.update_sensor_data(3, rpm);

        last_update = millis();
    }
}

void handle_channel_update(uint8_t channel, uint16_t value) {
    Serial.printf("Channel %d updated to %d\n", channel, value);
}

void handle_sensor_request(uint8_t sensor_id, uint8_t request_type) {
    Serial.printf("Sensor %d request: 0x%02X\n", sensor_id, request_type);
}

void handle_error(uint8_t error_code) {
    Serial.printf("Error: %d\n", error_code);
}

// Helper functions (implement based on your hardware)
int32_t read_temperature() { return 25; } // Example
int32_t read_rpm() { return 5000; } // Example
```

### Statistics Monitoring

```cpp
#include <IBus.h>
#include <Mf_Serial.h>

// Define pins for Arduino Mega Serial1
#define IBUS_RX_PIN 19    // RX1 pin for iBUS receiver
#define IBUS_TX_PIN -1    // TX1 pin not used (receive only)

Mf_Serial ibus_serial;
IBus ibus;

void setup() {
    Serial.begin(115200);
    ibus.begin(ibus_serial, IBus::NO_TIMER, IBUS_RX_PIN, IBUS_TX_PIN);
}

void loop() {
    ibus.process_events();

    // Print statistics every 5 seconds
    static uint32_t last_stats = 0;
    if (millis() - last_stats > 5000) {
        IBus::Statistics stats = ibus.get_statistics();

        Serial.println("=== IBus Statistics ===");
        Serial.printf("Messages received: %lu\n", stats.messages_received);
        Serial.printf("Messages processed: %lu\n", stats.messages_processed);
        Serial.printf("Sensor requests: %lu\n", stats.sensor_requests);
        Serial.printf("Sensor responses: %lu\n", stats.sensor_responses);
        Serial.printf("Checksum errors: %lu\n", stats.checksum_errors);
        Serial.printf("Protocol errors: %lu\n", stats.protocol_errors);
        Serial.printf("Active sensors: %d\n", ibus.get_active_sensor_count());
        Serial.println("=======================");

        last_stats = millis();
    }
}
```

## Architecture Overview

This library uses a completely different architectural approach compared to traditional implementations:

### Event-Driven Processing

- **Callback-based**: Uses std::function for flexible event handling
- **Real-time**: Immediate processing of incoming data
- **Non-blocking**: No polling delays or blocking operations

### Message Parser

- **Byte-by-byte**: Processes data one byte at a time
- **State machine**: Uses enum-based parsing states
- **Validation**: Built-in checksum and protocol validation

### Sensor Management

- **Dynamic registration**: Sensors can be added/removed at runtime
- **Flexible data types**: Support for 2-byte and 4-byte data
- **Automatic responses**: Handles all sensor protocol automatically

### Statistics and Monitoring

- **Comprehensive tracking**: Monitors all communication aspects
- **Error detection**: Identifies and reports protocol errors
- **Performance metrics**: Tracks message processing efficiency

## Protocol Details

The iBUS protocol is a half-duplex serial protocol operating at 115200 baud:

- **Frame format**: Length byte + Command byte + Data + Checksum
- **Channel data**: 14 channels, 2 bytes each (1000-2000 range)
- **Sensor support**: Up to 10 sensors with various data types
- **Checksum**: 16-bit subtraction-based checksum

## Compatibility

- **MadFlight Boards**: Optimized for MadFlight hardware
- **Mf_Serial**: Uses MadFlight's serial interface
- **Arduino IDE**: Compatible with Arduino development environment
- **C++11**: Requires C++11 or later for std::function support

## Contributing

Contributions are welcome! Please ensure your code follows the existing style and includes appropriate documentation.

## License

This library is released under the MIT License, providing maximum flexibility for both personal and commercial use.
