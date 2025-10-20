#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace gree {

enum ac_mode: uint8_t {
  AC_MODE_OFF = 0x10,
  AC_MODE_AUTO = 0x80,
  AC_MODE_COOL = 0x90,
  AC_MODE_DRY = 0xA0,
  AC_MODE_FANONLY = 0xB0,
  AC_MODE_HEAT = 0xC0
};

enum ac_fan: uint8_t {
  AC_FAN_AUTO = 0x00,
  AC_FAN_LOW = 0x01,
  AC_FAN_MEDIUM = 0x02,
  AC_FAN_HIGH = 0x03
};

// Bit 1 in Byte 10
enum display_mode: uint8_t {
  DISPLAY_OFF = 0x00,
  DISPLAY_ON = 0x02
};

// Byte 11
enum swing_mode_t: uint8_t {
  SWING_OFF = 0x60,
  SWING_ON = 0x10
};

// Bit 0 in Byte 10 (Turbo)
enum turbo_mode_t: uint8_t {
  TURBO_BIT = 0x01
};

#define GREE_START_BYTE 0x7E
#define GREE_RX_BUFFER_SIZE 52

union gree_start_bytes_t {
    uint8_t u8x2[2];
};

struct gree_header_t {
  gree_start_bytes_t start_bytes;
  uint8_t data_length;
};

struct gree_raw_packet_t {
  gree_header_t header;
  uint8_t data[1];
};

class GreeClimate : public climate::Climate, public uart::UARTDevice, public PollingComponent {
 public:
  void loop() override;
  void update() override;
  void dump_config() override;
  void control(const climate::ClimateCall &call) override;
  
  // Setters for switch control
  void set_display(bool state);
  void set_turbo(bool state);
  void set_swing(bool state);
  
  // YAML setters
  void set_turbo_switch(esphome::switch_::Switch *s) { this->turbo_switch = s; }
  void set_swing_switch(esphome::switch_::Switch *s) { this->swing_switch = s; }

  // Switch pointers
  esphome::switch_::Switch *turbo_switch{nullptr};
  esphome::switch_::Switch *swing_switch{nullptr};

  // State variables
  display_mode display_state_{DISPLAY_OFF};
  swing_mode_t swing_state_{SWING_OFF};
  bool turbo_state_ = false;
  
  // Saved temperature
  float saved_temperature{24.0f};

 protected:
  climate::ClimateTraits traits() override;
  void read_state_(const uint8_t *data, uint8_t size);
  void send_data_(const uint8_t *message, uint8_t size);
  void dump_message_(const char *title, const uint8_t *message, uint8_t size);
  uint8_t get_checksum_(const uint8_t *message, size_t size);
  void restore_state_();

 private:
  uint8_t data_write_[47] = {0x7E, 0x7E, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x0E, 0x60, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t data_read_[GREE_RX_BUFFER_SIZE] = {0};

  bool receiving_packet_{false};
  bool has_valid_state_{false};
  bool first_update_{true};
};

} // namespace gree
} // namespace esphome
