/*
 * MadFlight IBus Library
 *
 * A modern C++ library for handling Flysky/Turnigy RC iBUS protocol
 * Designed specifically for MadFlight boards using Mf_Serial
 *
 * MIT License
 *
 * Copyright (c) 2025 Atin M <atinm.dev@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>
#include "IBus.h"

// Static instance management (different approach)
IBus* IBus::m_primary_instance = nullptr;

/**
 * Timer interrupt handler for automatic processing
 */
void IBus::timer_interrupt_handler() {
    if (m_primary_instance) {
        m_primary_instance->process_events();
    }
}

/**
 * Constructor
 */
IBus::IBus()
    : m_serial_interface(nullptr)
    , m_registered_sensors(0)
    , m_data_available_flag(false)
    , m_last_activity_time(0) {

    // Initialize channel values to center position
    for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
        m_channel_values[i] = CHANNEL_CENTER;
    }

    // Initialize sensor registry
    for (uint8_t i = 0; i < MAX_SENSORS; ++i) {
        m_sensor_registry[i] = SensorEntry();
    }

    // Initialize statistics
    m_stats = {0, 0, 0, 0, 0, 0};

    // Reset parser
    m_parser.reset();

    // Link to primary instance
    m_primary_instance = this;
}

/**
 * Destructor
 */
IBus::~IBus() {
    if (m_primary_instance == this) {
        m_primary_instance = nullptr;
    }
}

/**
 * Initialize the IBus communication with event-driven processing
 */
void IBus::begin(Mf_Serial& serial, int8_t timer_id, int8_t rx_pin, int8_t tx_pin) {
    // Initialize the serial interface
    serial.begin(115200);
    m_serial_interface = &serial;

    // Reset parser and flags
    m_parser.reset();
    m_data_available_flag = false;
    m_last_activity_time = millis();

    // Reset statistics
    reset_statistics();

    // Note: Timer setup would be handled by MadFlight framework
    // For now, we rely on manual process_events() calls or external timer management
}

/**
 * Set callback for channel updates (event-driven approach)
 */
void IBus::on_channel_update(ChannelUpdateCallback callback) {
    m_channel_callback = callback;
}

/**
 * Set callback for sensor requests (event-driven approach)
 */
void IBus::on_sensor_request(SensorRequestCallback callback) {
    m_sensor_callback = callback;
}

/**
 * Set callback for error conditions
 */
void IBus::on_error(ErrorCallback callback) {
    m_error_callback = callback;
}

/**
 * Get current channel value (polling approach as alternative)
 */
uint16_t IBus::get_channel_value(uint8_t channel) const {
    if (!validate_channel_index(channel)) {
        return 0;
    }
    return m_channel_values[channel];
}

/**
 * Register a sensor for telemetry (different approach)
 */
bool IBus::register_sensor(uint8_t sensor_id, uint8_t sensor_type, uint8_t data_size) {
    if (!validate_sensor_id(sensor_id) || (data_size != 2 && data_size != 4)) {
        return false;
    }

    uint8_t index = sensor_id - 1;
    m_sensor_registry[index].type = sensor_type;
    m_sensor_registry[index].data_size = data_size;
    m_sensor_registry[index].is_registered = true;
    m_sensor_registry[index].last_update_time = millis();

    if (!m_sensor_registry[index].is_registered) {
        m_registered_sensors++;
    }

    return true;
}

/**
 * Update sensor data (different approach)
 */
bool IBus::update_sensor_data(uint8_t sensor_id, int32_t data) {
    if (!validate_sensor_id(sensor_id)) {
        return false;
    }

    uint8_t index = sensor_id - 1;
    if (m_sensor_registry[index].is_registered) {
        m_sensor_registry[index].current_value = data;
        m_sensor_registry[index].last_update_time = millis();
        return true;
    }

    return false;
}

/**
 * Process incoming data (event-driven processing)
 */
void IBus::process_events() {
    if (!m_serial_interface) {
        return;
    }

    // Process available data using event-driven approach
    while (m_serial_interface->available() > 0) {
        uint8_t byte = m_serial_interface->read();
        m_last_activity_time = millis();

        // Use parser to handle byte-by-byte processing
        MessageParser::ParseResult result = m_parser.parse_byte(byte);

        switch (result) {
            case MessageParser::ParseResult::COMPLETE:
                m_stats.messages_received++;
                if (m_parser.is_valid()) {
                    m_stats.messages_processed++;
                    process_complete_message();
                } else {
                    m_stats.checksum_errors++;
                    trigger_error(1); // Checksum error
                }
                m_parser.reset();
                break;

            case MessageParser::ParseResult::ERROR:
                m_stats.protocol_errors++;
                trigger_error(2); // Protocol error
                m_parser.reset();
                break;

            case MessageParser::ParseResult::INCOMPLETE:
                // Continue waiting for more data
                break;
        }
    }
}

/**
 * Check if new data is available (polling alternative)
 */
bool IBus::has_new_data() const {
    return m_data_available_flag;
}

/**
 * Get communication statistics
 */
IBus::Statistics IBus::get_statistics() const {
    return m_stats;
}

/**
 * Reset all statistics
 */
void IBus::reset_statistics() {
    m_stats = {0, 0, 0, 0, 0, 0};
}

/**
 * Get the number of active sensors
 */
uint8_t IBus::get_active_sensor_count() const {
    return m_registered_sensors;
}

/**
 * Process a complete message (event-driven approach)
 */
void IBus::process_complete_message() {
    uint8_t command = m_parser.get_command();
    uint8_t data_length = m_parser.get_data_length();
    const uint8_t* data = m_parser.get_data();

    if (command == IBUS_CMD_SERVO) {
        handle_servo_message(data, data_length);
    } else {
        handle_sensor_message(data, data_length);
    }
}

/**
 * Handle servo message (different approach)
 */
void IBus::handle_servo_message(const uint8_t* data, uint8_t length) {
    // Extract channel data using different approach
    uint8_t channels_to_process = (length - 1) / 2; // Each channel is 2 bytes
    if (channels_to_process > MAX_CHANNELS) {
        channels_to_process = MAX_CHANNELS;
    }

    for (uint8_t i = 0; i < channels_to_process; ++i) {
        uint16_t new_value = data[i * 2 + 1] | (data[i * 2 + 2] << 8);

        // Validate and update channel value
        if (new_value >= CHANNEL_MIN && new_value <= CHANNEL_MAX) {
            if (m_channel_values[i] != new_value) {
                m_channel_values[i] = new_value;
                m_data_available_flag = true;

                // Trigger callback if set
                if (m_channel_callback) {
                    m_channel_callback(i, new_value);
                }
            }
        }
    }
}

/**
 * Handle sensor message (different approach)
 */
void IBus::handle_sensor_message(const uint8_t* data, uint8_t length) {
    if (length != 1) {
        return; // Invalid sensor message length
    }

    uint8_t sensor_id = data[0] & 0x0F;
    uint8_t command_type = data[0] & 0xF0;

    if (!validate_sensor_id(sensor_id)) {
        return;
    }

    m_stats.sensor_requests++;

    // Trigger callback if set
    if (m_sensor_callback) {
        m_sensor_callback(sensor_id, command_type);
    }

    // Send appropriate response
    send_sensor_reply(sensor_id, command_type);
}

/**
 * Send sensor reply (different approach)
 */
void IBus::send_sensor_reply(uint8_t sensor_id, uint8_t command_type) {
    if (!m_serial_interface || !validate_sensor_id(sensor_id)) {
        return;
    }

    uint8_t index = sensor_id - 1;
    if (!m_sensor_registry[index].is_registered) {
        return;
    }

    const SensorEntry& sensor = m_sensor_registry[index];

    switch (command_type) {
        case IBUS_CMD_SENSOR_DISCOVER:
            // Send discover response
            m_serial_interface->write(0x04);
            m_serial_interface->write(IBUS_CMD_SENSOR_DISCOVER + sensor_id);
            m_serial_interface->write(0x7A);
            m_serial_interface->write(0xFF);
            break;

        case IBUS_CMD_SENSOR_TYPE:
            // Send sensor type response
            m_serial_interface->write(0x06);
            m_serial_interface->write(IBUS_CMD_SENSOR_TYPE + sensor_id);
            m_serial_interface->write(sensor.type);
            m_serial_interface->write(sensor.data_size);
            m_serial_interface->write(0x66);
            m_serial_interface->write(0xFF);
            break;

        case IBUS_CMD_SENSOR_DATA:
            // Send sensor data response
            m_stats.sensor_responses++;
            uint8_t response_length = 0x04 + sensor.data_size;
            m_serial_interface->write(response_length);
            m_serial_interface->write(IBUS_CMD_SENSOR_DATA + sensor_id);

            // Send data bytes (little-endian)
            m_serial_interface->write(sensor.current_value & 0xFF);
            m_serial_interface->write((sensor.current_value >> 8) & 0xFF);

            if (sensor.data_size == 4) {
                m_serial_interface->write((sensor.current_value >> 16) & 0xFF);
                m_serial_interface->write((sensor.current_value >> 24) & 0xFF);
            }

            // Send checksum (simplified)
            m_serial_interface->write(0x00);
            m_serial_interface->write(0x00);
            break;
    }
}

/**
 * Trigger error callback
 */
void IBus::trigger_error(uint8_t error_code) {
    if (m_error_callback) {
        m_error_callback(error_code);
    }
}

/**
 * Validate channel index
 */
bool IBus::validate_channel_index(uint8_t channel) const {
    return channel < MAX_CHANNELS;
}

/**
 * Validate sensor ID
 */
bool IBus::validate_sensor_id(uint8_t sensor_id) const {
    return sensor_id > 0 && sensor_id <= MAX_SENSORS;
}

// MessageParser implementation
IBus::MessageParser::ParseResult IBus::MessageParser::parse_byte(uint8_t byte) {
    if (m_position == 0) {
        // Start of new message
        if (byte <= IBUS_FRAME_SIZE && byte > IBUS_HEADER_SIZE) {
            m_expected_length = byte;
            m_frame_buffer[m_position++] = byte;
            m_calculated_checksum = 0xFFFF - byte;
            return ParseResult::INCOMPLETE;
        } else {
            return ParseResult::ERROR;
        }
    } else if (m_position < m_expected_length) {
        // Data bytes
        m_frame_buffer[m_position++] = byte;
        m_calculated_checksum -= byte;
        return ParseResult::INCOMPLETE;
    } else if (m_position == m_expected_length) {
        // First checksum byte
        m_received_checksum = byte;
        m_position++;
        return ParseResult::INCOMPLETE;
    } else {
        // Second checksum byte
        m_received_checksum |= (byte << 8);
        m_checksum_valid = (m_calculated_checksum == m_received_checksum);
        return ParseResult::COMPLETE;
    }
}

void IBus::MessageParser::reset() {
    m_position = 0;
    m_expected_length = 0;
    m_calculated_checksum = 0;
    m_received_checksum = 0;
    m_checksum_valid = false;
}

bool IBus::MessageParser::is_valid() const {
    return m_checksum_valid;
}

uint8_t IBus::MessageParser::get_command() const {
    return m_frame_buffer[0];
}

uint8_t IBus::MessageParser::get_data_length() const {
    return m_expected_length - 1; // Subtract command byte
}

const uint8_t* IBus::MessageParser::get_data() const {
    return &m_frame_buffer[1];
}

void IBus::MessageParser::calculate_checksum() {
    // Checksum calculation is done during parsing
}

