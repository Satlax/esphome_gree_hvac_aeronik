#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
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

enum display_mode: uint8_t {
  DISPLAY_OFF = 0x00,
  DISPLAY_ON = 0x02  // Бит 1 в байте 10
};

enum sound_mode: uint8_t {
  SOUND_ON = 0x00,   // Звук включен
  SOUND_OFF = 0x01   // Бит 0 в байте 11
};

enum turbo_mode: uint8_t {
  TURBO_OFF = 0x02,  // Нормальный режим
  TURBO_ON = 0x07    // Турбо режим (код 7 или 15)
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

 protected:
  climate::ClimateTraits traits() override;
  void read_state_(const uint8_t *data, uint8_t size);
  void send_data_(const uint8_t *message, uint8_t size);
  void dump_message_(const char *title, const uint8_t *message, uint8_t size);
  uint8_t get_checksum_(const uint8_t *message, size_t size);

 private:
  uint8_t data_write_[47] = {0x7E, 0x7E, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t data_read_[GREE_RX_BUFFER_SIZE] = {0};

  bool receiving_packet_ = false;
  bool has_valid_state_ = false;
  
  // Используем enum для более чистого кода
  display_mode display_state_ = DISPLAY_OFF;
  sound_mode sound_state_ = SOUND_ON;  // По умолчанию звук включен
  turbo_mode turbo_state_ = TURBO_OFF;
};

}  // namespace gree
}  // namespace esphome
