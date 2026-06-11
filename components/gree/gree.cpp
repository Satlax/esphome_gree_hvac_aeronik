#include "gree.h"
#include "esphome/core/log.h"

namespace esphome {
namespace gree {

static const char *TAG = "gree";

// =========================
// CONFIG
// =========================
void GreeClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "Gree climate loaded");
}

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
    receiving_packet_ = true;
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
// STATE PARSE
// =========================
void GreeClimate::read_state_(const uint8_t *data, uint8_t size) {
  if (data[size - 1] != get_checksum_(data, size))
    return;

  if (data[3] != 49)
    return;

  this->target_temperature =
      data[TEMPERATURE] / 16 + MIN_TEMP;

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

  this->display_state_ = data[DISPLAY_BYTE] & DISPLAY_BIT;
  this->turbo_state_ = (data[TURBO_BYTE] == 7 || data[TURBO_BYTE] == 15);

  this->publish_state();
}

// =========================
// CONTROL
// =========================
void GreeClimate::control(const climate::ClimateCall &call) {
  uint8_t mode = data_write_[MODE] & MODE_MASK;
  uint8_t fan  = data_write_[MODE] & FAN_MASK;

  if (call.get_mode().has_value()) {
    switch (call.get_mode().value()) {
      case climate::CLIMATE_MODE_OFF:  mode = AC_MODE_OFF; break;
      case climate::CLIMATE_MODE_AUTO: mode = AC_MODE_AUTO; break;
      case climate::CLIMATE_MODE_COOL: mode = AC_MODE_COOL; break;
      case climate::CLIMATE_MODE_DRY:  mode = AC_MODE_DRY; break;
      case climate::CLIMATE_MODE_FAN_ONLY: mode = AC_MODE_FAN; break;
      case climate::CLIMATE_MODE_HEAT: mode = AC_MODE_HEAT; break;
      default: break;
    }
  }

  if (call.get_fan_mode().has_value()) {
    switch (call.get_fan_mode().value()) {
      case climate::CLIMATE_FAN_AUTO: fan = AC_FAN_AUTO; break;
      case climate::CLIMATE_FAN_LOW: fan = AC_FAN_LOW; break;
      case climate::CLIMATE_FAN_MEDIUM: fan = AC_FAN_MEDIUM; break;
      case climate::CLIMATE_FAN_HIGH: fan = AC_FAN_HIGH; break;
      default: break;
    }
  }

  data_write_[MODE] = mode | fan;

  if (call.get_target_temperature().has_value()) {
    float t = call.get_target_temperature().value();
    if (t >= MIN_TEMP && t <= MAX_TEMP)
      data_write_[TEMPERATURE] = (t - MIN_TEMP) * 16;
  }

  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  this->write_array(data_write_, sizeof(data_write_));
}

// =========================
// DISPLAY
// =========================
void GreeClimate::set_display(bool state) {
  display_state_ = state;

  if (state)
    data_write_[DISPLAY_BYTE] |= DISPLAY_BIT;
  else
    data_write_[DISPLAY_BYTE] &= ~DISPLAY_BIT;

  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  this->write_array(data_write_, sizeof(data_write_));
}

// =========================
// TURBO
// =========================
void GreeClimate::set_turbo(bool state) {
  turbo_state_ = state;

  uint8_t mode = data_write_[MODE] & MODE_MASK;

  if (mode == AC_MODE_HEAT)
    data_write_[TURBO_BYTE] = state ? 15 : 14;
  else
    data_write_[TURBO_BYTE] = state ? 7 : 6;

  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  this->write_array(data_write_, sizeof(data_write_));
}

// =========================
// UPDATE
// =========================
void GreeClimate::update() {
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
  return sum;
}

}  // namespace gree
}  // namespace esphome
