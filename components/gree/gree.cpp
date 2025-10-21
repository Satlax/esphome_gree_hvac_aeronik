#include <cmath>
#include "gree.h"
#include "esphome/core/macros.h"

namespace esphome {
namespace gree {

static const char *const TAG = "gree";

// byte index constants (match your dumps)
static const uint8_t FORCE_UPDATE = 7;
static const uint8_t MODE = 8;
static const uint8_t MODE_MASK = 0b11110000;
static const uint8_t FAN_MASK = 0b00001111;
static const uint8_t CRC_WRITE = 46;
static const uint8_t TEMPERATURE = 9;
static const uint8_t INDOOR_TEMPERATURE = 46;

static const uint8_t MIN_VALID_TEMPERATURE = 16;
static const uint8_t MAX_VALID_TEMPERATURE = 30;
static const uint8_t TEMPERATURE_STEP = 1;

void GreeClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "Gree:");
  ESP_LOGCONFIG(TAG, "  Update interval: %u", this->get_update_interval());
  this->dump_traits_(TAG);
  this->check_uart_settings(4800, 1, uart::UART_CONFIG_PARITY_EVEN, 8);
}

void GreeClimate::loop() {
  gree_raw_packet_t *raw_packet = (gree_raw_packet_t *)this->data_read_;

  while (!receiving_packet_ && this->available() >= sizeof(gree_header_t)) {
    if (this->peek() != GREE_START_BYTE) {
      this->read();
      continue;
    }

    this->read_array(this->data_read_, sizeof(gree_start_bytes_t));
    receiving_packet_ = (raw_packet->header.start_bytes.u8x2[1] == GREE_START_BYTE);

    if (receiving_packet_) {
      this->read_byte(&raw_packet->header.data_length);
      if (raw_packet->header.data_length + sizeof(gree_header_t) > GREE_RX_BUFFER_SIZE) {
        ESP_LOGE(TAG, "Incoming packet is too big! header.data_length = %d", raw_packet->header.data_length);
        receiving_packet_ = false;
        memset(this->data_read_, 0, GREE_RX_BUFFER_SIZE);
      }
    }
  }

  if (receiving_packet_ && this->available() >= raw_packet->header.data_length) {
    this->read_array(raw_packet->data, raw_packet->header.data_length);
    dump_message_("Read array", this->data_read_, raw_packet->header.data_length + sizeof(gree_header_t));
    read_state_(this->data_read_, raw_packet->header.data_length + sizeof(gree_header_t));
    receiving_packet_ = false;
    memset(this->data_read_, 0, GREE_RX_BUFFER_SIZE);
  }
}

void GreeClimate::update() {
  // Only send periodic state if we already know valid state (prevents spamming at boot).
  if (!this->has_valid_state_) return;
  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  send_data_(data_write_, sizeof(data_write_));
}

climate::ClimateTraits GreeClimate::traits() {
  auto traits = climate::ClimateTraits();

  traits.set_visual_min_temperature(MIN_VALID_TEMPERATURE);
  traits.set_visual_max_temperature(MAX_VALID_TEMPERATURE);
  traits.set_visual_temperature_step(TEMPERATURE_STEP);

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

  // Presets: use preset to represent Swing positions
  traits.set_supported_presets({
    climate::CLIMATE_PRESET_NONE,    // swing off
    climate::CLIMATE_PRESET_BOOST    // reuse BOOST for "swing full" (we will map others via values)
  });
  // Note: additional presets in ESPhome are limited to known enums.
  // We will map swing variations in code and expose one preset (BOOST) as indicator for full swing,
  // while other swing positions will be handled internally and reflected in climate attributes/state textually.

  traits.set_supports_current_temperature(true);
  traits.set_supports_two_point_target_temperature(false);

  return traits;
}

void GreeClimate::read_state_(const uint8_t *data, uint8_t size) {
  // validate checksum
  const uint8_t data_crc = data[size - 1];
  if (data_crc != get_checksum_(data, size)) {
    ESP_LOGW(TAG, "Invalid checksum.");
    return;
  }

  if (data[3] != 49) {
    ESP_LOGW(TAG, "Invalid packet type.");
    return;
  }

  // parse display bit (byte 10)
  this->display_on_ = (data[10] & DISPLAY_BIT) != 0;

  // parse turbo bit
  this->turbo_on_ = (data[10] & TURBO_BIT) != 0;

  // parse swing from byte 11 (observed mapping)
  uint8_t swing_byte = data[11];
  // match known values (some devices use particular mask/values)
  if (swing_byte == SWING_FULL) this->swing_value_ = SWING_FULL;
  else if (swing_byte == SWING_TOP) this->swing_value_ = SWING_TOP;
  else if (swing_byte == SWING_MIDDLE) this->swing_value_ = SWING_MIDDLE;
  else if (swing_byte == SWING_BOTTOM) this->swing_value_ = SWING_BOTTOM;
  else this->swing_value_ = SWING_OFF;

  // temperatures
  this->target_temperature = (float)(data[TEMPERATURE] / 16) + (float)MIN_VALID_TEMPERATURE;
  this->current_temperature = (float)(data[INDOOR_TEMPERATURE]) - 40.0f;

  // save bytes for re-sending
  data_write_[MODE] = data[MODE];
  data_write_[TEMPERATURE] = data[TEMPERATURE];

  // update climate mode
  switch (data[MODE] & MODE_MASK) {
    case AC_MODE_OFF: this->mode = climate::CLIMATE_MODE_OFF; break;
    case AC_MODE_AUTO: this->mode = climate::CLIMATE_MODE_AUTO; break;
    case AC_MODE_COOL: this->mode = climate::CLIMATE_MODE_COOL; break;
    case AC_MODE_DRY: this->mode = climate::CLIMATE_MODE_DRY; break;
    case AC_MODE_FANONLY: this->mode = climate::CLIMATE_MODE_FAN_ONLY; break;
    case AC_MODE_HEAT: this->mode = climate::CLIMATE_MODE_HEAT; break;
    default:
      ESP_LOGW(TAG, "Unknown AC mode: 0x%02X", data[MODE]);
      break;
  }

  // update fan mode
  switch (data[MODE] & FAN_MASK) {
    case AC_FAN_AUTO: this->fan_mode = climate::CLIMATE_FAN_AUTO; break;
    case AC_FAN_LOW: this->fan_mode = climate::CLIMATE_FAN_LOW; break;
    case AC_FAN_MEDIUM: this->fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
    case AC_FAN_HIGH: this->fan_mode = climate::CLIMATE_FAN_HIGH; break;
    default:
      ESP_LOGW(TAG, "Unknown fan mode: 0x%02X", data[MODE] & FAN_MASK);
      this->fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
  }

  // represent swing in preset: we use BOOST for "swing full", NONE otherwise.
  if (this->swing_value_ == SWING_FULL) this->preset = climate::CLIMATE_PRESET_BOOST;
  else this->preset = climate::CLIMATE_PRESET_NONE;

  // Publish states for linked switches so UI updates if remote used
  if (this->display_switch_ != nullptr) {
    this->display_switch_->publish_state(this->display_on_);
  }
  if (this->turbo_switch_ != nullptr) {
    this->turbo_switch_->publish_state(this->turbo_on_);
  }

  this->has_valid_state_ = true;
  this->publish_state();
}

void GreeClimate::control(const climate::ClimateCall &call) {
  // always mark we want to force update on ac
  data_write_[FORCE_UPDATE] = 175;

  // start from previously-known mode/fan
  uint8_t new_mode = data_write_[MODE] & MODE_MASK;
  uint8_t new_fan_speed = data_write_[MODE] & FAN_MASK;

  // set mode if requested
  if (call.get_mode().has_value()) {
    switch (call.get_mode().value()) {
      case climate::CLIMATE_MODE_OFF: new_mode = AC_MODE_OFF; break;
      case climate::CLIMATE_MODE_AUTO: new_mode = AC_MODE_AUTO; break;
      case climate::CLIMATE_MODE_COOL: new_mode = AC_MODE_COOL; break;
      case climate::CLIMATE_MODE_DRY: new_mode = AC_MODE_DRY; new_fan_speed = AC_FAN_LOW; break;
      case climate::CLIMATE_MODE_FAN_ONLY: new_mode = AC_MODE_FANONLY; break;
      case climate::CLIMATE_MODE_HEAT: new_mode = AC_MODE_HEAT; break;
      default: break;
    }
  }

  // fan speed
  if (call.get_fan_mode().has_value()) {
    switch (call.get_fan_mode().value()) {
      case climate::CLIMATE_FAN_AUTO: new_fan_speed = AC_FAN_AUTO; break;
      case climate::CLIMATE_FAN_LOW: new_fan_speed = AC_FAN_LOW; break;
      case climate::CLIMATE_FAN_MEDIUM: new_fan_speed = AC_FAN_MEDIUM; break;
      case climate::CLIMATE_FAN_HIGH: new_fan_speed = AC_FAN_HIGH; break;
      default: break;
    }
  }

  // preset used for swing selection (we map preset -> swing_value_)
  if (call.get_preset().has_value()) {
    auto p = call.get_preset().value();
    // we only have 2 built-in presets enums in traits: NONE and BOOST used for "swing full"
    if (p == climate::CLIMATE_PRESET_NONE) {
      // user selected none: set swing off
      this->swing_value_ = SWING_OFF;
    } else if (p == climate::CLIMATE_PRESET_BOOST) {
      // map BOOST to swing full
      this->swing_value_ = SWING_FULL;
    }
  }

  // custom logic: target temperature
  if (call.get_target_temperature().has_value()) {
    float temp = call.get_target_temperature().value();
    if (temp >= MIN_VALID_TEMPERATURE && temp <= MAX_VALID_TEMPERATURE) {
      data_write_[TEMPERATURE] = uint8_t((temp - MIN_VALID_TEMPERATURE) * 16);
    }
  }

  // assemble mode byte
  data_write_[MODE] = new_mode | new_fan_speed;

  // byte 10: manage turbo and display bits, keep other bits intact
  // clear turbo+display bits:
  data_write_[10] &= ~uint8_t(TURBO_BIT | DISPLAY_BIT);

  if (this->turbo_on_) data_write_[10] |= TURBO_BIT;
  if (this->display_on_) data_write_[10] |= DISPLAY_BIT;

  // byte 11: set swing value directly (observed mapping)
  data_write_[11] = this->swing_value_;

  // finalize and send
  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  send_data_(data_write_, sizeof(data_write_));

  data_write_[FORCE_UPDATE] = 0;

  // publish updated state immediately
  this->publish_state();
}

void GreeClimate::set_display(bool on) {
  this->display_on_ = on;
  // publish to linked switch immediately if present
  if (this->display_switch_ != nullptr) this->display_switch_->publish_state(on);
  // call control to apply change to AC (use an empty ClimateCall parent)
  this->control(this->make_call());
}

void GreeClimate::set_turbo(bool on) {
  this->turbo_on_ = on;
  if (this->turbo_switch_ != nullptr) this->turbo_switch_->publish_state(on);
  this->control(this->make_call());
}

void GreeClimate::send_data_(const uint8_t *message, uint8_t size) {
  this->write_array(message, size);
  dump_message_("Sent message", message, size);
}

void GreeClimate::dump_message_(const char *title, const uint8_t *message, uint8_t size) {
  ESP_LOGV(TAG, "%s:", title);
  char str[256] = {0};
  char *p = str;
  for (uint8_t i = 0; i < size && (p - str) < (int)(sizeof(str) - 4); ++i) {
    p += sprintf(p, "%02X ", message[i]);
  }
  ESP_LOGV(TAG, "%s", str);
}

uint8_t GreeClimate::get_checksum_(const uint8_t *message, size_t size) {
  uint8_t sum = 0;
  // sum bytes excluding first two start bytes and the last CRC byte
  for (size_t i = 2; i < size - 1; ++i) sum += message[i];
  return uint8_t(sum % 256);
}

}  // namespace gree
}  // namespace esphome
