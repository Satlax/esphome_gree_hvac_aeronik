#include "gree.h"
#include "esphome/core/log.h"

namespace esphome {
namespace gree {

static const char *TAG = "gree";

// =========================
// LOOP
// =========================
void GreeClimate::loop() {
  gree_raw_packet_t *raw = (gree_raw_packet_t *) this->data_read_;

  while (!receiving_packet_ && this->available() >= sizeof(gree_header_t)) {
    if (this->peek() != GREE_START_BYTE) {
      this->read();
      continue;
    }

    this->read_array(this->data_read_, sizeof(gree_header_t));
    receiving_packet_ = (raw->header.start_bytes.u8x2[1] == GREE_START_BYTE);

    if (receiving_packet_) {
      this->read_byte(&raw->header.data_length);
    }
  }

  if (receiving_packet_ &&
      this->available() >= raw->header.data_length) {

    this->read_array(raw->data, raw->header.data_length);

    read_state_(this->data_read_,
                raw->header.data_length + sizeof(gree_header_t));

    receiving_packet_ = false;
  }
}

// =========================
// STATE PARSE (REVERSED CORE)
// =========================
void GreeClimate::read_state_(const uint8_t *data, uint8_t size) {
  if (data[size - 1] != get_checksum_(data, size))
    return;

  if (data[3] != 49)
    return;

  // TEMP
  this->target_temperature =
      data[TEMPERATURE] / 16 + MIN_VALID_TEMPERATURE;

  this->current_temperature =
      (data[INDOOR_TEMPERATURE] >= 40)
        ? data[INDOOR_TEMPERATURE] - 40
        : 0;

  // MODE
  switch (data[MODE] & MODE_MASK) {
    case AC_MODE_OFF:  this->mode = climate::CLIMATE_MODE_OFF; break;
    case AC_MODE_AUTO: this->mode = climate::CLIMATE_MODE_AUTO; break;
    case AC_MODE_COOL: this->mode = climate::CLIMATE_MODE_COOL; break;
    case AC_MODE_DRY:  this->mode = climate::CLIMATE_MODE_DRY; break;
    case AC_MODE_FAN:  this->mode = climate::CLIMATE_MODE_FAN_ONLY; break;
    case AC_MODE_HEAT: this->mode = climate::CLIMATE_MODE_HEAT; break;
  }

  // FAN
  switch (data[MODE] & FAN_MASK) {
    case AC_FAN_AUTO:   this->fan_mode = climate::CLIMATE_FAN_AUTO; break;
    case AC_FAN_LOW:    this->fan_mode = climate::CLIMATE_FAN_LOW; break;
    case AC_FAN_MEDIUM: this->fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
    case AC_FAN_HIGH:   this->fan_mode = climate::CLIMATE_FAN_HIGH; break;
  }

  // SWING
  switch (data[SWING]) {
    case AC_SWING_OFF:    this->preset = climate::CLIMATE_PRESET_NONE; break;
    case AC_SWING_FULL:   this->preset = climate::CLIMATE_PRESET_BOOST; break;
    case AC_SWING_TOP:    this->preset = climate::CLIMATE_PRESET_ECO; break;
    case AC_SWING_MIDDLE:  this->preset = climate::CLIMATE_PRESET_AWAY; break;
    case AC_SWING_BOTTOM:  this->preset = climate::CLIMATE_PRESET_SLEEP; break;
  }

  // DISPLAY STATE
  this->display_state_ = (data[DISPLAY_BYTE] & DISPLAY_BIT);

  // TURBO STATE (confirmed behavior)
  uint8_t t = data[TURBO_BYTE];
  this->turbo_state_ = (t == 7 || t == 15);

  this->publish_state();
}

// =========================
// DISPLAY CONTROL
// =========================
void GreeClimate::set_display(bool state) {
  this->display_state_ = state;

  if (state)
    data_write_[DISPLAY_BYTE] |= DISPLAY_BIT;
  else
    data_write_[DISPLAY_BYTE] &= ~DISPLAY_BIT;

  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  this->write_array(data_write_, sizeof(data_write_));
}

// =========================
// TURBO CONTROL
// =========================
void GreeClimate::set_turbo(bool state) {
  this->turbo_state_ = state;

  uint8_t mode = data_write_[MODE] & MODE_MASK;

  if (mode == AC_MODE_HEAT)
    data_write_[TURBO_BYTE] = state ? 15 : 14;
  else
    data_write_[TURBO_BYTE] = state ? 7 : 6;

  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  this->write_array(data_write_, sizeof(data_write_));
}

// =========================
// CHECKSUM
// =========================
uint8_t GreeClimate::get_checksum_(const uint8_t *msg, size_t size) {
  uint8_t sum = 0;
  for (int i = 2; i < size - 1; i++)
    sum += msg[i];
  return sum % 256;
}

// =========================
// DUMP
// =========================
void GreeClimate::dump_message_(const char *title, const uint8_t *msg, uint8_t size) {
  ESP_LOGV(TAG, "%s", title);
}

void GreeClimate::send_data_(const uint8_t *msg, uint8_t size) {
  this->write_array(msg, size);
}

}  // namespace gree
}  // namespace esphome
