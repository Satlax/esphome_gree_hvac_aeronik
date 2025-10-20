#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/log.h"
#include "esphome/components/switch/switch.h"

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

// Константы для управления Свингом (Базируются на байте 13)
enum ac_swing_preset : uint8_t {
  SWING_OFF = 0x00,
  SWING_FULL = 0x10,
  SWING_TOP = 0x20,
  SWING_MID = 0x40,
  SWING_LOW = 0x60
};

// Константы для управления Display и Turbo (Базируются на байте 10)
// Используем эти константы как базовые маски, которые будут дополняться.
static const uint8_t DISPLAY_ON_MASK = 0x0A; // Предполагаемая база для ON (может быть 0x0E)
static const uint8_t DISPLAY_OFF_MASK = 0x08;
static const uint8_t TURBO_BIT_COOL = 0x07;
static const uint8_t TURBO_BIT_HEAT = 0x0F;
static const uint8_t HEAT_MODE_OFFSET = 0x08; // Сдвиг для режима HEAT (например, 0x0E -> 0x16)

#define GREE_START_BYTE 0x7E
#define GREE_RX_BUFFER_SIZE 52

union gree_start_bytes_t {
    uint8_t u8x2[2];
};

struct gree_header_t
{
  gree_start_bytes_t start_bytes;
  uint8_t data_length;
};

struct gree_raw_packet_t
{
  gree_header_t header;
  uint8_t data[1]; // first data byte
};


class GreeClimate : public climate::Climate, public uart::UARTDevice, public PollingComponent {
 public:
  void loop() override;
  void update() override;
  void dump_config() override;
  void control(const climate::ClimateCall &call) override;
  void set_supported_presets(const std::set<climate::ClimatePreset> &presets) { this->supported_presets_ = presets; }
  
  // Функции для управления новыми переключателями
  void set_display(bool state);
  void set_turbo(bool state);

  // Функции для связывания переключателей с компонентом
  void set_display_switch(switch_::Switch *s) { this->display_switch_ = s; }
  void set_turbo_switch(switch_::Switch *s) { this->turbo_switch_ = s; }

 protected:
  climate::ClimateTraits traits() override;
  void read_state_(const uint8_t *data, uint8_t size);
  void send_data_(const uint8_t *message, uint8_t size);
  void dump_message_(const char *title, const uint8_t *message, uint8_t size);
  uint8_t get_checksum_(const uint8_t *message, size_t size);

 private:
  // Parts of the message that must have specific values for "send" command.
  uint8_t data_write_[47] = {0x7E, 0x7E, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t data_read_[GREE_RX_BUFFER_SIZE] = {0};

  bool receiving_packet_ = false;

  // Новые состояния для переключателей
  bool display_state_ = false;
  bool turbo_state_ = false;
  uint8_t current_swing_value_ = SWING_OFF; // Для хранения текущего байта Swing

  switch_::Switch *display_switch_{nullptr};
  switch_::Switch *turbo_switch_{nullptr};

  std::set<climate::ClimatePreset> supported_presets_{};
};

// Классы для создания переключателей в YAML
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


}  // namespace gree
}  // namespace esphome
