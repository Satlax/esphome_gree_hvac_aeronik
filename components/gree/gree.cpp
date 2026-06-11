#include "gree.h"
#include "esphome/core/log.h"

namespace esphome {
namespace gree {

static const char *TAG = "gree";

// =========================
// CONFIG DUMP (REQUIRED)
// =========================
void GreeClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "Gree Climate:");
  this->dump_traits_(TAG);
}

// =========================
// TRAITS (REQUIRED)
// =========================
climate::ClimateTraits GreeClimate::traits() {
  auto t = climate::ClimateTraits();

  t.set_visual_min_temperature(MIN_VALID_TEMPERATURE);
  t.set_visual_max_temperature(MAX_VALID_TEMPERATURE);
  t.set_visual_temperature_step(TEMPERATURE_STEP);

  t.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_AUTO,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_DRY,
    climate::CLIMATE_MODE_FAN_ONLY,
    climate::CLIMATE_MODE_HEAT
  });

  t.set_supported_fan_modes({
    climate::CLIMATE_FAN_AUTO,
    climate::CLIMATE_FAN_LOW,
    climate::CLIMATE_FAN_MEDIUM,
    climate::CLIMATE_FAN_HIGH
  });

  t.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);

  for (auto &p : this->supported_presets_)
    t.add_supported_preset(p);

  t.add_supported_preset(climate::CLIMATE_PRESET_NONE);

  return t;
}

// =========================
// UPDATE (REQUIRED)
// =========================
void GreeClimate::update() {
  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  this->write_array(data_write_, sizeof(data_write_));
}

// =========================
// CONTROL (REQUIRED)
// =========================
void GreeClimate::control(const climate::ClimateCall &call) {
  data_write_[FORCE_UPDATE] = 175;

  uint8_t mode = data_write_[MODE] & MODE_MASK;
  uint8_t fan  = data_write_[MODE] & FAN_MASK;

  if (call.get_mode().has_value()) {
    switch (call.get_mode().value()) {
      case climate::CLIMATE_MODE_OFF:  mode = AC_MODE_OFF; break;
      case climate::CLIMATE_MODE_AUTO: mode = AC_MODE_AUTO; break;
      case climate::CLIMATE_MODE_COOL: mode = AC_MODE_COOL; break;
      case climate::CLIMATE_MODE_DRY:  mode = AC_MODE_DRY; fan = AC_FAN_LOW; break;
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
    if (t >= MIN_VALID_TEMPERATURE && t <= MAX_VALID_TEMPERATURE)
      data_write_[TEMPERATURE] = (t - MIN_VALID_TEMPERATURE) * 16;
  }

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

// =========================
// LOG DUMP
// =========================
void GreeClimate::dump_message_(const char *title, const uint8_t *msg, uint8_t size) {
  ESP_LOGV(TAG, "%s", title);
}

// =========================
// SEND
// =========================
void GreeClimate::send_data_(const uint8_t *msg, uint8_t size) {
  this->write_array(msg, size);
}

}  // namespace gree
}  // namespace esphome
