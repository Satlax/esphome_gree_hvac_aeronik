#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/log.h"

namespace esphome {
namespace gree {

// AC modes
enum ac_mode: uint8_t {
  AC_MODE_OFF = 0x10,
  AC_MODE_AUTO = 0x80,
  AC_MODE_COOL = 0x90,
  AC_MODE_DRY = 0xA0,
  AC_MODE_FANONLY = 0xB0,
  AC_MODE_HEAT = 0xC0
};

// AC fan speeds
enum ac_fan: uint8_t {
  AC_FAN_AUTO = 0x00,
  AC_FAN_LOW = 0x01,
  AC_FAN_MEDIUM = 0x02,
  AC_FAN_HIGH = 0x03
};

// Display / Sound / Turbo encodings (used as bit masks / codes)
enum display_mode: uint8_t {
  DISPLAY_OFF = 0x00,
  DISPLAY_ON  = 0x02  // bit 1 in byte 10
};

enum sound_mode: uint8_t {
  SOUND_ON  = 0x00,
  SOUND_OFF = 0x01  // bit 0 in byte 11
};

enum turbo_mode: uint8_t {
  TURBO_OFF = 0x00,
  TURBO_ON_CODE = 0x07 // value 7 in byte 10 indicates turbo on in some modes (we also check 15)
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
  uint8_t data[1]; // first data byte
};

class GreeClimate : public climate::Climate, public uart::UARTDevice, public PollingComponent {
 public:
  void loop() override;
  void update() override;
  void dump_config() override;
  void control(const climate::ClimateCall &call) override;

  // (optional) getters if you want to expose via lambda
  bool is_turbo_enabled() const { return turbo_enabled_; }
  bool is_display_enabled() const { return display_enabled_; }
  bool is_sound_enabled() const { return sound_enabled_; }

 protected:
  climate::ClimateTraits traits() override;
  void read_state_(const uint8_t *data, uint8_t size);
  void send_data_(const uint8_t *message, uint8_t size);
  void dump_message_(const char *title, const uint8_t *message, uint8_t size);
  uint8_t get_checksum_(const uint8_t *message, size_t size);

 private:
  // base packet to send (47 bytes as in original)
  uint8_t data_write_[47] = {
    0x7E, 0x7E, 0x2C, 0x01,
    0x00,0x00,0x00,0x00,  // 4..7
    0x00,0x00,0x02,0x02,  // 8..11 (byte10 index 10 used for display/preset)
    0x00,0x00,0x00,0x00,  // 12..15
    0x00,0x00,0x00,0x00,  // 16..19
    0x00,0x00,0x00,0x00,  // 20..23
    0x00,0x00,0x00,0x00,  // 24..27
    0x00,0x00,0x00,0x00,  // 28..31
    0x00,0x00,0x00,0x00,  // 32..35
    0x00,0x00,0x00,0x00,  // 36..39
    0x00,0x00,0x00 // rest up to 47; CRC at index 46 will be set before send
  };
  uint8_t data_read_[GREE_RX_BUFFER_SIZE] = {0};

  bool receiving_packet_ = false;
  bool has_valid_state_ = false;

  // internal logical flags (persist in RAM)
  bool turbo_enabled_ = false;   // Turbo ON/OFF
  bool display_enabled_ = true;  // Display ON/OFF
  bool sound_enabled_ = true;    // Sound ON/OFF (true = sound enabled)

  // saved parsed states (for clarity)
  display_mode parsed_display_ = DISPLAY_ON;
  sound_mode parsed_sound_ = SOUND_ON;
  turbo_mode parsed_turbo_ = TURBO_OFF;
};

}  // namespace gree
}  // namespace esphome
