#include <IBus.h>
#include <Mf_Serial.h>
#include <Servo.h>

/*
  Convert iBUS data to Servo PWM format using event-driven processing
  Reads data from first servo channel and translates this to PWM signal to send to a normal servo

  Hardware: Arduino Mega
  - Serial1 RX pin (19) connected to iBUS receiver pin
  - Servo connected to pin 9

  This example demonstrates real-time servo control using the new event-driven API.
*/

// Define pins for Arduino Mega Serial1
#define IBUS_RX_PIN 19    // RX1 pin for iBUS receiver
#define IBUS_TX_PIN -1    // TX1 pin not used for servo control (receive only)

Mf_Serial ibus_serial;
IBus ibus;           // IBus object
Servo myservo;       // create servo object to control a servo

// Global variables for tracking
uint32_t servo_updates = 0;
uint32_t last_debug_time = 0;

void setup() {
  // Initialize serial for debug output
  Serial.begin(115200);
  Serial.println("iBUS to Servo PWM converter started (Event-Driven) - Arduino Mega");

  // Initialize iBUS with event-driven processing and specific pins
  ibus.begin(ibus_serial, IBus::NO_TIMER, IBUS_RX_PIN, IBUS_TX_PIN);

  // Attach the servo on pin 9 to the servo object
  myservo.attach(9);

  // Set up event callback for real-time servo control
  ibus.on_channel_update([](uint8_t channel, uint16_t value) {
    if (channel == 0) {  // Only process channel 0 for servo control
      servo_updates++;

      // Set the servo position using the iBUS value directly
      // iBUS values are typically 1000-2000 microseconds, which is perfect for servo.writeMicroseconds()
      myservo.writeMicroseconds(value);

      // Optional: Print debug information for channel 0
      Serial.printf("Channel 0: %d (Update #%lu)\n", value, servo_updates);
    }
  });

  ibus.on_error([](uint8_t error_code) {
    Serial.printf("Error: %d\n", error_code);
  });

  Serial.println("Event handlers set up. Waiting for channel data...");
}

void loop() {
  // Process events (call this regularly)
  ibus.process_events();

  // Optional: Display statistics periodically
  static uint32_t last_stats_time = 0;
  if (millis() - last_stats_time > 5000) {  // Every 5 seconds
    IBus::Statistics stats = ibus.get_statistics();
    Serial.println("=== Statistics ===");
    Serial.printf("Messages received: %lu\n", stats.messages_received);
    Serial.printf("Messages processed: %lu\n", stats.messages_processed);
    Serial.printf("Servo updates: %lu\n", servo_updates);
    Serial.printf("Current channel 0 value: %d\n", ibus.get_channel_value(0));
    Serial.printf("Errors: %lu\n", stats.checksum_errors + stats.protocol_errors);
    Serial.println("==================");

    last_stats_time = millis();
  }
}

