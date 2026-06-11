#pragma once

#include <set>

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace gree {

#define GREE_START_BYTE 0x7E
#define GREE_RX_BUFFER_SIZE 52

enum ac_mode : uint8_t {
  AC_MODE_OFF  = 0x10,
  AC_MODE_AUTO = 0x80,
  AC_MODE_COOL = 0x90,
  AC_MODE_DRY  = 0xA0,
  AC_MODE_FAN  = 0xB0,
  AC_MODE_HEAT = 0xC0
};

enum ac_fan : uint8_t {
  AC_FAN_AUTO   = 0x00,
  AC_FAN_LOW    = 0x01,
  AC_FAN_MEDIUM = 0x02,
  AC_FAN_HIGH   = 0x03
};

enum ac_swing : uint8_t {
  AC_SWING_OFF    = 0x00,
  AC_SWING_FULL   = 0x10,
  AC_SWING_TOP    = 0x20,
  AC_SWING_MIDDLE = 0x40,
  AC_SWING_BOTTOM = 0x60
};

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

class GreeClimate : public climate::Climate,
                    public uart::UARTDevice,
                    public PollingComponent {
 public:
  void loop() override;
  void update() override;
  void dump_config() override;
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

  void set_supported_presets(const std::set<climate::ClimatePreset> &presets) {
    supported_presets_ = presets;
  }

  void set_display(bool state);
  void set_turbo(bool state);

  bool get_display_state() const { return display_state_; }
  bool get_turbo_state() const { return turbo_state_; }

 protected:
  void read_state_(const uint8_t *data, uint8_t size);
  uint8_t get_checksum_(const uint8_t *message, size_t size);

 private:
  static const uint8_t FORCE_UPDATE = 7;
  static const uint8_t MODE = 8;
  static const uint8_t MODE_MASK = 0b11110000;
  static const uint8_t FAN_MASK  = 0b00001111;

  static const uint8_t TEMPERATURE = 9;
  static const uint8_t SWING = 11;

  static const uint8_t DISPLAY_BYTE = 13;
  static const uint8_t DISPLAY_BIT  = 0x20;

  static const uint8_t TURBO_BYTE = 10;

  static const uint8_t INDOOR_TEMPERATURE = 45;
  static const uint8_t CRC_WRITE = 46;

  static const uint8_t MIN_TEMP = 16;
  static const uint8_t MAX_TEMP = 30;

  bool display_state_{false};
  bool turbo_state_{false};

  uint8_t data_write_[47] = {
    0x7E,0x7E,0x2C,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
    0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00
  };

  uint8_t data_read_[GREE_RX_BUFFER_SIZE] = {0};
  bool receiving_packet_{false};
  uint32_t last_rx_time_{0};

  std::set<climate::ClimatePreset> supported_presets_{};
};

}  // namespace gree
}  // namespace esphome
