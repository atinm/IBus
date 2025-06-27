#include <IBus.h>
#include <Mf_Serial.h>

/*
  Simulate two sensors and send information back over the iBUS to the receiver (and back to transmitter
  as telemetry).

  Hardware: Arduino Mega
  - Serial - monitor output (debug output to PC, this is through the build-in USB)
  - Serial1 - connected to the ibus receiver pin (RX1=19, TX1=18)
    Connect the TX1 pin also to the RX1/ibus connection using an 1.2k Ohm resistor or 1N4148 diode
    (cathode=white ring of the diode at the side of TX1)

  This example demonstrates the new event-driven sensor API with callback-based processing.
*/

// Define pins for Arduino Mega Serial1
#define IBUS_RX_PIN 19    // RX1 pin for iBUS receiver
#define IBUS_TX_PIN 18    // TX1 pin for iBUS (used for sensor telemetry)

Mf_Serial ibus_serial;
IBus ibus;

// Sensor values
uint16_t speed = 0;
uint16_t temp = 420; // start at 20°C (base 400 = -40°C)

// Statistics tracking
uint32_t sensor_requests = 0;
uint32_t sensor_responses = 0;

void setup() {
  // Initialize serial port for debug
  Serial.begin(115200);

  // Initialize iBUS with event-driven processing and specific pins
  ibus.begin(ibus_serial, IBus::NO_TIMER, IBUS_RX_PIN, IBUS_TX_PIN);

  Serial.println("Start iBUS sensor (Event-Driven) - Arduino Mega");

  // Set up event callbacks
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

  Serial.println("Sensors registered. Waiting for requests...");
}

void loop() {
  // Process events (call this regularly)
  ibus.process_events();

  // Update sensor data periodically
  static uint32_t last_update = 0;
  if (millis() - last_update > 500) {
    speed += 10;  // increase motor speed by 10 RPM
    temp++;       // increase temperature by 0.1°C

    ibus.update_sensor_data(1, speed);
    ibus.update_sensor_data(2, temp);

    // Display current values
    Serial.printf("Speed: %d RPM, Temp: %.1f°C\n", speed, (temp - 400) / 10.0);

    // Show statistics
    IBus::Statistics stats = ibus.get_statistics();
    Serial.printf("Sensor requests: %lu, Responses: %lu\n",
                  stats.sensor_requests, stats.sensor_responses);

    last_update = millis();
  }
}

