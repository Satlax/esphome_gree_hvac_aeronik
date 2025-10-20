#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace gree {

static const uint8_t GREE_START_BYTE = 0x5a;

enum ac_mode : uint8_t {
  AC_MODE_OFF = 0x10,
  AC_MODE_AUTO = 0x80,
  AC_MODE_COOL = 0x90,
  AC_MODE_DRY = 0xA0,
  AC_MODE_FANONLY = 0xB0,
  AC_MODE_HEAT = 0xC0
};

enum ac_fan : uint8_t {
  AC_FAN_AUTO = 0x00,
  AC_FAN_LOW = 0x01,
  AC_FAN_MEDIUM = 0x02,
  AC_FAN_HIGH = 0x03
};

// Mask for display
static const uint8_t DISPLAY_ON_MASK = 0x07;
static const uint8_t DISPLAY_OFF_MASK = 0x05;

// Turbo bits in Byte 10
static const uint8_t TURBO_BIT_COOL = 0x0f;
static const uint8_t TURBO_BIT_HEAT = 0x17;
static const uint8_t HEAT_MODE_OFFSET = 0x08;

// Swing values (Byte 13)
static const uint8_t SWING_OFF = 0x20;
static const uint8_t SWING_FULL = 0x26;
static const uint8_t SWING_TOP = 0x21;
static const uint8_t SWING_MID = 0x22;
static const uint8_t SWING_LOW = 0x24;


struct gree_start_bytes_t {
  uint8_t u8x2[2];
};

struct gree_header_t {
  gree_start_bytes_t start_bytes;
  uint8_t data_length;
};

struct gree_raw_packet_t {
  gree_header_t header;
  uint8_t data[1]; // first data byte
};

static const uint8_t GREE_RX_BUFFER_SIZE = 64;

class TurboSwitch;
class DisplaySwitch;

class GreeClimate : public climate::Climate, public uart::UARTDevice, public PollingComponent {
 public:
  void dump_config() override;
  void loop() override;
  void update() override;
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;
  
  void set_turbo_switch(TurboSwitch *s) { this->turbo_switch_ = s; }
  void set_display_switch(DisplaySwitch *s) { this->display_switch_ = s; }
  
  // Custom functions for switches (Implementation in .cpp)
  void set_display(bool state);
  void set_turbo(bool state);
  
  // Public states for switches (used by lambda in YAML)
  bool turbo_state_ = false;
  bool display_state_ = false;
  
 protected:
  void read_state_(const uint8_t *data, uint8_t size);
  void send_data_(const uint8_t *message, uint8_t size);
  void dump_message_(const char *title, const uint8_t *message, uint8_t size);
  uint8_t get_checksum_(const uint8_t *message, size_t size);

  uint8_t data_read_[GREE_RX_BUFFER_SIZE] = {0};
  uint8_t data_write_[47] = {0x5a, 0x5a, 0x01, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  bool receiving_packet_ = false;

  uint8_t current_swing_value_ = SWING_OFF;

  TurboSwitch *turbo_switch_ = nullptr;
  DisplaySwitch *display_switch_ = nullptr;
};

// Classes for creating switches in YAML
class DisplaySwitch : public switch_::Switch {
 public:
  DisplaySwitch(GreeClimate *parent) : parent_(parent) {}
  void write_state(bool state) override { this->parent_->set_display(state); }
 protected:
  GreeClimate *parent_;
};

class TurboSwitch : public switch_::Switch {
 public:
  TurboSwitch(GreeClimate *parent) : parent_(parent) {}
  void write_state(bool state) override { this->parent_->set_turbo(state); }
 protected:
  GreeClimate *parent_;
};


} // namespace gree
} // namespace esphome
