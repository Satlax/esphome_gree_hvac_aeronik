#include "gree.h"
#include "esphome/core/log.h"

namespace esphome {
namespace gree {

static const char *TAG = "gree";

void GreeClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "Gree climate loaded");
  ESP_LOGCONFIG(TAG, "  Update interval: %u ms", this->get_update_interval());
  this->check_uart_settings(4800, 1, uart::UART_CONFIG_PARITY_EVEN, 8);
}

climate::ClimateTraits GreeClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_visual_min_temperature(MIN_TEMP);
  traits.set_visual_max_temperature(MAX_TEMP);
  traits.set_visual_temperature_step(1.0f);

  traits.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_AUTO,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_DRY,
    climate::CLIMATE_MODE_FAN_ONLY,
    climate::CLIMATE_MODE_HEAT
  });

  traits.set_supported_fan_modes({
    climate::CLIMATE_FAN_AUTO,
    climate::CLIMATE_FAN_LOW,
    climate::CLIMATE_FAN_MEDIUM,
    climate::CLIMATE_FAN_HIGH
  });

  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);

  for (auto preset : supported_presets_)
    traits.add_supported_preset(preset);
  traits.add_supported_preset(climate::CLIMATE_PRESET_NONE);

  return traits;
}

void GreeClimate::loop() {
  yield();  // сброс WDT

  gree_raw_packet_t *raw = (gree_raw_packet_t *) this->data_read_;

  while (!receiving_packet_ && this->available() >= sizeof(gree_header_t)) {
    if (this->peek() != GREE_START_BYTE) {
      this->read();
      continue;
    }
    this->read_array(this->data_read_, sizeof(gree_header_t));
    receiving_packet_ = true;
    last_rx_time_ = millis();
  }

  if (receiving_packet_) {
    if (this->available() >= raw->header.data_length) {
      this->read_array(raw->data, raw->header.data_length);
      size_t total_size = raw->header.data_length + sizeof(gree_header_t);
      read_state_(this->data_read_, total_size);
      receiving_packet_ = false;
    } else if (millis() - last_rx_time_ > 500) {
      ESP_LOGW(TAG, "Packet reception timeout");
      receiving_packet_ = false;
      memset(this->data_read_, 0, GREE_RX_BUFFER_SIZE);
    }
  }
}

void GreeClimate::read_state_(const uint8_t *data, uint8_t size) {
  if (size < INDOOR_TEMPERATURE + 1) {
    ESP_LOGW(TAG, "Packet too short: %d bytes", size);
    return;
  }

  if (data[size - 1] != get_checksum_(data, size))
    return;
  if (data[3] != 49)
    return;

  this->target_temperature = data[TEMPERATURE] / 16 + MIN_TEMP;

  int indoor = data[INDOOR_TEMPERATURE];
  if (indoor >= 40)
    this->current_temperature = indoor - 40;
  else
    this->current_temperature = 0;

  uint8_t mode_byte = data[MODE];
  switch (mode_byte & MODE_MASK) {
    case AC_MODE_OFF:  this->mode = climate::CLIMATE_MODE_OFF; break;
    case AC_MODE_AUTO: this->mode = climate::CLIMATE_MODE_AUTO; break;
    case AC_MODE_COOL: this->mode = climate::CLIMATE_MODE_COOL; break;
    case AC_MODE_DRY:  this->mode = climate::CLIMATE_MODE_DRY; break;
    case AC_MODE_FAN:  this->mode = climate::CLIMATE_MODE_FAN_ONLY; break;
    case AC_MODE_HEAT: this->mode = climate::CLIMATE_MODE_HEAT; break;
    default: break;
  }

  switch (mode_byte & FAN_MASK) {
    case AC_FAN_AUTO:   this->fan_mode = climate::CLIMATE_FAN_AUTO; break;
    case AC_FAN_LOW:    this->fan_mode = climate::CLIMATE_FAN_LOW; break;
    case AC_FAN_MEDIUM: this->fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
    case AC_FAN_HIGH:   this->fan_mode = climate::CLIMATE_FAN_HIGH; break;
    default: break;
  }

  this->display_state_ = (data[DISPLAY_BYTE] & DISPLAY_BIT) != 0;
  uint8_t turbo_val = data[TURBO_BYTE];
  this->turbo_state_ = (turbo_val == 7 || turbo_val == 15);

  this->publish_state();
}

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

void GreeClimate::set_display(bool state) {
  display_state_ = state;
  if (state)
    data_write_[DISPLAY_BYTE] |= DISPLAY_BIT;
  else
    data_write_[DISPLAY_BYTE] &= ~DISPLAY_BIT;
  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  this->write_array(data_write_, sizeof(data_write_));
}

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

void GreeClimate::update() {
  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  this->write_array(data_write_, sizeof(data_write_));
}

uint8_t GreeClimate::get_checksum_(const uint8_t *msg, size_t size) {
  uint8_t sum = 0;
  for (size_t i = 2; i < size - 1; i++)
    sum += msg[i];
  return sum;   // <--- КЛЮЧЕВОЕ ИСПРАВЛЕНИЕ: возвращаем сумму
}

}  // namespace gree
}  // namespace esphome
