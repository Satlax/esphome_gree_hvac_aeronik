#include <cmath>
#include "gree.h"
#include "esphome/core/macros.h"

namespace esphome {
namespace gree {

static const char *const TAG = "gree";

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
      this->read(); // throw away
      continue;
    }

    this->read_array(this->data_read_, sizeof(gree_start_bytes_t));
    receiving_packet_ = (raw_packet->header.start_bytes.u8x2[1] == GREE_START_BYTE);

    if (receiving_packet_) {
      this->read_byte(&raw_packet->header.data_length);

      if (raw_packet->header.data_length + sizeof(gree_header_t) > GREE_RX_BUFFER_SIZE) {
        ESP_LOGE(TAG, "Incoming packet is too big! header.data_length = %d, maximum is %d", raw_packet->header.data_length, GREE_RX_BUFFER_SIZE - sizeof(gree_header_t));
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

  traits.set_supports_current_temperature(true);
  traits.set_supports_two_point_target_temperature(false);

  // supported presets passed from YAML (if any)
  traits.set_supported_presets(this->supported_presets_);

  // ensure at least NONE is available
  traits.add_supported_preset(climate::CLIMATE_PRESET_NONE);

  return traits;
}

void GreeClimate::read_state_(const uint8_t *data, uint8_t size) {
  // checksum validation
  uint8_t data_crc = data[size-1];
  uint8_t get_crc = get_checksum_(data, size);

  if (data_crc != get_crc) {
    ESP_LOGW(TAG, "Invalid checksum.");
    return;
  }

  if (data[3] != 49) {
    ESP_LOGW(TAG, "Invalid packet type.");
    return;
  }

  this->target_temperature = data[TEMPERATURE] / 16 + MIN_VALID_TEMPERATURE;
  this->current_temperature = data[INDOOR_TEMPERATURE] - 40;

  // partially saving current state to previous request
  data_write_[MODE] = data[MODE];
  data_write_[TEMPERATURE] = data[TEMPERATURE];

  // update CLIMATE mode
  switch (data[MODE] & MODE_MASK) {
    case AC_MODE_OFF:
      this->mode = climate::CLIMATE_MODE_OFF;
      break;
    case AC_MODE_AUTO:
      this->mode = climate::CLIMATE_MODE_AUTO;
      break;
    case AC_MODE_COOL:
      this->mode = climate::CLIMATE_MODE_COOL;
      break;
    case AC_MODE_DRY:
      this->mode = climate::CLIMATE_MODE_DRY;
      break;
    case AC_MODE_FANONLY:
      this->mode = climate::CLIMATE_MODE_FAN_ONLY;
      break;
    case AC_MODE_HEAT:
      this->mode = climate::CLIMATE_MODE_HEAT;
      break;
    default:
      ESP_LOGW(TAG, "Unknown AC MODE&fan: %02X", data[MODE]);
  }

  // fan speed
  switch (data[MODE] & FAN_MASK) {
    case AC_FAN_AUTO:
      this->fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
    case AC_FAN_LOW:
      this->fan_mode = climate::CLIMATE_FAN_LOW;
      break;
    case AC_FAN_MEDIUM:
      this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
      break;
    case AC_FAN_HIGH:
      this->fan_mode = climate::CLIMATE_FAN_HIGH;
      break;
    default:
      ESP_LOGW(TAG, "Unknown AC fan: %02X", data[MODE]);
  }

  // map swing byte to preset
  uint8_t swing_byte = data[SWING];
  switch (swing_byte) {
    case AC_SWING_OFF:
      this->preset = climate::CLIMATE_PRESET_NONE;
      break;
    case AC_SWING_FULL:
      this->preset = climate::CLIMATE_PRESET_BOOST;  // full sweep -> BOOST
      break;
    case AC_SWING_TOP:
      this->preset = climate::CLIMATE_PRESET_ECO;    // top -> ECO (chosen mapping)
      break;
    case AC_SWING_MIDDLE:
      this->preset = climate::CLIMATE_PRESET_AWAY;   // middle -> AWAY (chosen mapping)
      break;
    case AC_SWING_BOTTOM:
      this->preset = climate::CLIMATE_PRESET_SLEEP;  // bottom -> SLEEP
      break;
    default:
      this->preset = climate::CLIMATE_PRESET_NONE;
      break;
  }

  // detect turbo byte (data[10]) and set preset BOOST as well if turbo is on
  // NOTE: we don't override swing-mapped preset; this only sets BOOST if turbo active and no swing significance
  if (data[10] == 7 || data[10] == 15) {
    // turbo active
    // keep existing preset but optionally mark BOOST if you want to represent turbo specifically
    // this->preset = climate::CLIMATE_PRESET_BOOST;
  }

  this->publish_state();
}

void GreeClimate::control(const climate::ClimateCall &call) {
  data_write_[FORCE_UPDATE] = 175;
  data_write_[13] = 0x20; // ensure display shows temperature while controlling (as before)

  // preserve current mode/fan bits
  uint8_t new_mode = data_write_[MODE] & MODE_MASK;
  uint8_t new_fan_speed = data_write_[MODE] & FAN_MASK;

  if (call.get_mode().has_value()) {
    switch (call.get_mode().value()) {
      case climate::CLIMATE_MODE_OFF:
        new_mode = AC_MODE_OFF;
        break;
      case climate::CLIMATE_MODE_AUTO:
        new_mode = AC_MODE_AUTO;
        break;
      case climate::CLIMATE_MODE_COOL:
        new_mode = AC_MODE_COOL;
        break;
      case climate::CLIMATE_MODE_DRY:
        new_mode = AC_MODE_DRY;
        new_fan_speed = AC_FAN_LOW;
        break;
      case climate::CLIMATE_MODE_FAN_ONLY:
        new_mode = AC_MODE_FANONLY;
        break;
      case climate::CLIMATE_MODE_HEAT:
        new_mode = AC_MODE_HEAT;
        break;
      default:
        ESP_LOGW(TAG, "Setting of unsupported MODE: %d", (int)call.get_mode().value());
        break;
    }
  }

  if (call.get_fan_mode().has_value()) {
    switch (call.get_fan_mode().value()) {
      case climate::CLIMATE_FAN_AUTO:
        new_fan_speed = AC_FAN_AUTO;
        break;
      case climate::CLIMATE_FAN_LOW:
        new_fan_speed = AC_FAN_LOW;
        break;
      case climate::CLIMATE_FAN_MEDIUM:
        new_fan_speed = AC_FAN_MEDIUM;
        break;
      case climate::CLIMATE_FAN_HIGH:
        new_fan_speed = AC_FAN_HIGH;
        break;
      default:
        ESP_LOGW(TAG, "Setting of unsupported FANSPEED: %d", (int)call.get_fan_mode().value());
        break;
    }
  }

  if (new_mode == AC_MODE_DRY && new_fan_speed != AC_FAN_LOW) {
    new_fan_speed = AC_FAN_LOW;
  }

  // presets -> swing mapping (we moved swing to presets)
  if (call.get_preset().has_value()) {
    switch (call.get_preset().value()) {
      case climate::CLIMATE_PRESET_NONE:
        data_write_[SWING] = AC_SWING_OFF;
        break;
      case climate::CLIMATE_PRESET_BOOST:
        data_write_[SWING] = AC_SWING_FULL;
        break;
      case climate::CLIMATE_PRESET_ECO:
        data_write_[SWING] = AC_SWING_TOP;
        break;
      case climate::CLIMATE_PRESET_AWAY:
        data_write_[SWING] = AC_SWING_MIDDLE;
        break;
      case climate::CLIMATE_PRESET_SLEEP:
        data_write_[SWING] = AC_SWING_BOTTOM;
        break;
      default:
        // any other -> switch off
        data_write_[SWING] = AC_SWING_OFF;
        break;
    }
  }

  if (call.get_target_temperature().has_value()) {
    if (call.get_target_temperature().value() >= MIN_VALID_TEMPERATURE && call.get_target_temperature().value() <= MAX_VALID_TEMPERATURE)
      data_write_[TEMPERATURE] = (call.get_target_temperature().value() - MIN_VALID_TEMPERATURE) * 16;
  }

  // swing handled above through preset mapping (SWING byte)

  data_write_[MODE] = new_mode + new_fan_speed;

  // compute checksum & send data
  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  send_data_(data_write_, sizeof(data_write_));

  // change of force_update byte to "passive" state
  data_write_[FORCE_UPDATE] = 0;
}

// send helpers
void GreeClimate::send_data_(const uint8_t *message, uint8_t size) {
  this->write_array(message, size);
  dump_message_("Sent message", message, size);
}

void GreeClimate::dump_message_(const char *title, const uint8_t *message, uint8_t size) {
  ESP_LOGV(TAG, "%s:", title);
  char str[250] = {0};
  char *pstr = str;
  if (size * 2 > sizeof(str)) ESP_LOGE(TAG, "too long byte data");
  for (int i = 0; i < size; i++) {
    pstr += sprintf(pstr, "%02X ", message[i]);
  }
  ESP_LOGV(TAG, "%s", str);
}

uint8_t GreeClimate::get_checksum_(const uint8_t *message, size_t size) {
  uint8_t position = size - 1;
  uint8_t sum = 0;
  for (int i = 2; i < position; i++)
    sum += message[i];
  uint8_t crc = sum % 256;
  return crc;
}

// new: set display on/off (byte 13, bit 0x20 was detected in logs)
void GreeClimate::set_display(bool state) {
  if (state)
    data_write_[13] = data_write_[13] | 0x20;
  else
    data_write_[13] = data_write_[13] & (~0x20);

  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  send_data_(data_write_, sizeof(data_write_));
}

// new: set turbo on/off (byte 10 uses 6/7 for cool, 14/15 for heat according to logs)
void GreeClimate::set_turbo(bool state) {
  uint8_t mode_only = data_write_[MODE] & MODE_MASK;
  if (mode_only == AC_MODE_COOL) {
    data_write_[10] = state ? 7 : 6;
  } else if (mode_only == AC_MODE_HEAT) {
    data_write_[10] = state ? 15 : 14;
  } else {
    // for other modes make best-effort: set to cooldown turbo (7) or default (6)
    data_write_[10] = state ? 7 : 6;
  }
  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  send_data_(data_write_, sizeof(data_write_));
}

}  // namespace gree
}  // namespace esphome
