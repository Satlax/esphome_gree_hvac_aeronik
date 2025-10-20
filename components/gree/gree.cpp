#include <cmath>
#include "gree.h"
#include "esphome/core/macros.h"

namespace esphome {
namespace gree {

static const char *const TAG = "gree";

// block of byte positions in requests/answers
static const uint8_t FORCE_UPDATE = 7;
static const uint8_t MODE = 8;
static const uint8_t MODE_MASK = 0b11110000;
static const uint8_t FAN_MASK = 0b00001111;
static const uint8_t SWING = 12;

static const uint8_t CRC_WRITE = 46;
static const uint8_t TEMPERATURE = 9;
static const uint8_t INDOOR_TEMPERATURE = 46;

// component settings
static const uint8_t MIN_VALID_TEMPERATURE = 16;
static const uint8_t MAX_VALID_TEMPERATURE = 30;
static const uint8_t TEMPERATURE_STEP = 1;

void GreeClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "Gree:");
  ESP_LOGCONFIG(TAG, "  Update interval: %u", this->get_update_interval());
  this->dump_traits_(TAG);
  this->check_uart_settings(4800, 1, uart::UART_CONFIG_PARITY_EVEN, 8);
}

void GreeClimate::setup() {
  ESP_LOGI(TAG, "Sending initial handshake to AC...");
  send_data_(data_write_, sizeof(data_write_));
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
        ESP_LOGE(TAG, "Incoming packet too big! header.data_length = %d, max = %d",
                 raw_packet->header.data_length, GREE_RX_BUFFER_SIZE - sizeof(gree_header_t));
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
  if (!has_valid_state_) {
    ESP_LOGW(TAG, "Skipping update: no valid state from AC yet");
    return;
  }
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
  traits.set_supported_presets(this->supported_presets_);
  traits.add_supported_preset(climate::CLIMATE_PRESET_NONE);
  traits.add_supported_preset(climate::CLIMATE_PRESET_BOOST);

  return traits;
}

void GreeClimate::read_state_(const uint8_t *data, uint8_t size) {
  if (size < 10) {
    ESP_LOGW(TAG, "Invalid packet (too short): %d bytes", size);
    return;
  }

  uint8_t calc_crc = get_checksum_(data, size);
  uint8_t recv_crc = data[size - 1];
  if (calc_crc != recv_crc) {
    ESP_LOGW(TAG, "Checksum mismatch: calc=0x%02X recv=0x%02X", calc_crc, recv_crc);
    dump_message_("Bad packet", data, size);
    return;
  }

  if (data[3] != 49) {
    ESP_LOGW(TAG, "Invalid packet type.");
    return;
  }

  bool new_light_state = data[10] & 0x02;
  this->display_light_state_ = new_light_state;

  this->target_temperature = data[TEMPERATURE] / 16 + MIN_VALID_TEMPERATURE;
  this->current_temperature = data[INDOOR_TEMPERATURE] - 40;

  data_write_[MODE] = data[MODE];
  data_write_[TEMPERATURE] = data[TEMPERATURE];

  switch (data[MODE] & MODE_MASK) {
    case AC_MODE_OFF: this->mode = climate::CLIMATE_MODE_OFF; break;
    case AC_MODE_AUTO: this->mode = climate::CLIMATE_MODE_AUTO; break;
    case AC_MODE_COOL: this->mode = climate::CLIMATE_MODE_COOL; break;
    case AC_MODE_DRY: this->mode = climate::CLIMATE_MODE_DRY; break;
    case AC_MODE_FANONLY: this->mode = climate::CLIMATE_MODE_FAN_ONLY; break;
    case AC_MODE_HEAT: this->mode = climate::CLIMATE_MODE_HEAT; break;
    default: ESP_LOGW(TAG, "Unknown AC MODE&fan: %s", data[MODE]);
  }

  switch (data[MODE] & FAN_MASK) {
    case AC_FAN_AUTO: this->fan_mode = climate::CLIMATE_FAN_AUTO; break;
    case AC_FAN_LOW: this->fan_mode = climate::CLIMATE_FAN_LOW; break;
    case AC_FAN_MEDIUM: this->fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
    case AC_FAN_HIGH: this->fan_mode = climate::CLIMATE_FAN_HIGH; break;
    default: ESP_LOGW(TAG, "Unknown AC mode&FAN: %s", data[MODE]);
  }

  switch (data[10]) {
    case 7: case 15: this->preset = climate::CLIMATE_PRESET_BOOST; break;
    default: this->preset = climate::CLIMATE_PRESET_NONE; break;
  }

  ESP_LOGI(TAG, "Diagnostic: data[10]=0x%02X -> display_bit=%d (reported by AC)",
           data[10], new_light_state ? 1 : 0);

  // сохраняем состояние дисплея в data_write_
  if (this->display_light_state_) data_write_[10] |= 0x02;
  else data_write_[10] &= ~0x02;

  this->publish_state();
  this->has_valid_state_ = true;
}

void GreeClimate::control(const climate::ClimateCall &call) {
  data_write_[FORCE_UPDATE] = 175;

  uint8_t new_mode = data_write_[MODE] & MODE_MASK;
  uint8_t new_fan_speed = data_write_[MODE] & FAN_MASK;

  if (call.get_mode().has_value()) {
    switch (call.get_mode().value()) {
      case climate::CLIMATE_MODE_OFF: new_mode = AC_MODE_OFF; break;
      case climate::CLIMATE_MODE_AUTO: new_mode = AC_MODE_AUTO; break;
      case climate::CLIMATE_MODE_COOL: new_mode = AC_MODE_COOL; break;
      case climate::CLIMATE_MODE_DRY: new_mode = AC_MODE_DRY; new_fan_speed = AC_FAN_LOW; break;
      case climate::CLIMATE_MODE_FAN_ONLY: new_mode = AC_MODE_FANONLY; break;
      case climate::CLIMATE_MODE_HEAT: new_mode = AC_MODE_HEAT; break;
      default: ESP_LOGW(TAG, "Unsupported MODE"); break;
    }
  }

  if (call.get_fan_mode().has_value()) {
    switch (call.get_fan_mode().value()) {
      case climate::CLIMATE_FAN_AUTO: new_fan_speed = AC_FAN_AUTO; break;
      case climate::CLIMATE_FAN_LOW: new_fan_speed = AC_FAN_LOW; break;
      case climate::CLIMATE_FAN_MEDIUM: new_fan_speed = AC_FAN_MEDIUM; break;
      case climate::CLIMATE_FAN_HIGH: new_fan_speed = AC_FAN_HIGH; break;
      default: ESP_LOGW(TAG, "Unsupported FAN SPEED"); break;
    }
  }

  if (new_mode == AC_MODE_DRY && new_fan_speed != AC_FAN_LOW) new_fan_speed = AC_FAN_LOW;

  if (call.get_preset().has_value()) {
    switch (call.get_preset().value()) {
      case climate::CLIMATE_PRESET_NONE:
        if (new_mode == AC_MODE_COOL) data_write_[10] = 6;
        else if (new_mode == AC_MODE_HEAT) data_write_[10] = 14;
        break;
      case climate::CLIMATE_PRESET_BOOST:
        if (new_mode == AC_MODE_COOL) data_write_[10] = 7;
        else if (new_mode == AC_MODE_HEAT) data_write_[10] = 15;
        break;
      default: break;
    }
  }

  if (call.get_target_temperature().has_value()) {
    float t = call.get_target_temperature().value();
    if (t >= MIN_VALID_TEMPERATURE && t <= MAX_VALID_TEMPERATURE)
      data_write_[TEMPERATURE] = (t - MIN_VALID_TEMPERATURE) * 16;
  }

  if (this->display_light_state_) data_write_[10] |= 0x02;
  else data_write_[10] &= ~0x02;

  data_write_[MODE] = new_mode + new_fan_speed;
  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  send_data_(data_write_, sizeof(data_write_));
  data_write_[FORCE_UPDATE] = 0;
}

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
  for (int i = 2; i < position; i++) sum += message[i];
  return sum % 256;
}

void GreeClimate::set_display_light(bool state) {
  this->display_light_state_ = state;
  ESP_LOGI(TAG, "Display light set to: %s", state ? "ON" : "OFF");
  this->control(this->make_call());
}

}  // namespace gree
}  // namespace esphome
