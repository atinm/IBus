#include <IBus.h>
#include <Mf_Serial.h>

/*
  Monitor iBUS signals and show output on the serial monitor for receivers with a single ibus pin
  such as the Flysky FS-iA8X (from specs, not tested yet). The TGY-IA6C also has
  one iBUS pin but only supports servo control signals and does not support external telemetry sensors.

  Hardware: Arduino Mega
  - Serial - monitor output (debug output to PC, this is through the build-in USB)
  - Serial1 - connected to the ibus receiver pin (RX1=19, TX1=18)

  Hardware connections to setup/test if you have a receiver with 1 ibus pins:
  1. Only connect the RX1 pin (19) to the ibus pin of the receiver
     --> you should see the servo values on screen
  2. Connect the TX1 pin (18) also to the RX1/ibus connection using an 1.2k Ohm reistor or 1N4148 diode
     (cathode=white ring of the diode at the side of TX1)
     --> dummy sensor data should be sent back to the receiver (sensor responses also change value)

  This example demonstrates the new event-driven API with callback-based processing.
*/

// Define pins for Arduino Mega Serial1
#define IBUS_RX_PIN 19    // RX1 pin for iBUS receiver
#define IBUS_TX_PIN 18    // TX1 pin for iBUS (used for sensor telemetry)

Mf_Serial ibus_serial;
IBus ibus;

// Global variables for tracking
uint32_t channel_updates = 0;
uint32_t sensor_requests = 0;
uint32_t last_display_time = 0;

// Sensor values
uint16_t speed = 0;
uint16_t temp = 420; // start at 20°C (base 400 = -40°C)

void setup() {
  // Initialize serial port for debug
  Serial.begin(115200);

  // Initialize iBUS with event-driven processing and specific pins
  ibus.begin(ibus_serial, IBus::NO_TIMER, IBUS_RX_PIN, IBUS_TX_PIN);

  Serial.println("Start iBUS monitor (Event-Driven) - Arduino Mega");

  // Set up event callbacks
  ibus.on_channel_update([](uint8_t channel, uint16_t value) {
    channel_updates++;
    // Optional: Print individual channel updates
    // Serial.printf("Channel %d: %d\n", channel, value);
  });

  ibus.on_sensor_request([](uint8_t sensor_id, uint8_t request_type) {
    sensor_requests++;
    Serial.printf("Sensor %d request: 0x%02X\n", sensor_id, request_type);
  });

  ibus.on_error([](uint8_t error_code) {
    Serial.printf("Error: %d\n", error_code);
  });

  // Register sensors for telemetry
  ibus.register_sensor(1, IBus::SENSOR_RPM, 2);
  ibus.register_sensor(2, IBus::SENSOR_TEMPERATURE, 2);

  Serial.println("Sensors registered. Waiting for data...");
}

void loop() {
  // Process events (call this regularly)
  ibus.process_events();

  // Update sensor data periodically
  static uint32_t last_sensor_update = 0;
  if (millis() - last_sensor_update > 500) {
    speed += 10;  // increase motor speed by 10 RPM
    temp++;       // increase temperature by 0.1°C

    ibus.update_sensor_data(1, speed);
    ibus.update_sensor_data(2, temp);

    last_sensor_update = millis();
  }

  // Display status every second
  if (millis() - last_display_time > 1000) {
    Serial.println("=== iBUS Status ===");

    // Show first 8 servo channels
    Serial.print("Channels: ");
    for (int i = 0; i < 8; i++) {
      Serial.print(ibus.get_channel_value(i));
      Serial.print(" ");
    }
    Serial.println();

    // Show statistics
    IBus::Statistics stats = ibus.get_statistics();
    Serial.printf("Messages received: %lu\n", stats.messages_received);
    Serial.printf("Messages processed: %lu\n", stats.messages_processed);
    Serial.printf("Channel updates: %lu\n", channel_updates);
    Serial.printf("Sensor requests: %lu\n", stats.sensor_requests);
    Serial.printf("Sensor responses: %lu\n", stats.sensor_responses);
    Serial.printf("Checksum errors: %lu\n", stats.checksum_errors);
    Serial.printf("Protocol errors: %lu\n", stats.protocol_errors);
    Serial.printf("Active sensors: %d\n", ibus.get_active_sensor_count());

    // Show current sensor values
    Serial.printf("Speed: %d RPM, Temp: %.1f°C\n", speed, (temp - 400) / 10.0);
    Serial.println("==================");

    last_display_time = millis();
  }
}

