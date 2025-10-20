#include "gree.h"
#include "esphome/core/log.h"

namespace esphome {
namespace gree {

static const char *const TAG = "gree";

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
        ESP_LOGE(TAG, "Incoming packet too big! header.data_length = %d", raw_packet->header.data_length);
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
  if (!has_valid_state_) return;
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

  // добавляем встроенные пресеты для управления светом и бипером
  traits.add_supported_preset(climate::CLIMATE_PRESET_NONE);
  traits.add_supported_preset(climate::CLIMATE_PRESET_BOOST);
  traits.add_supported_preset(climate::CLIMATE_PRESET_COMFORT); // можно использовать для Display Light
  traits.add_supported_preset(climate::CLIMATE_PRESET_ECO);     // можно использовать для Silent Mode

  traits.set_supported_presets(this->supported_presets_);

  return traits;
}

void GreeClimate::read_state_(const uint8_t *data, uint8_t size) {
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

  display_light_state_ = data[10] & 0x02;
  target_temperature = data[TEMPERATURE] / 16 + MIN_VALID_TEMPERATURE;
  current_temperature = data[INDOOR_TEMPERATURE] - 40;

  data_write_[MODE] = data[MODE];
  data_write_[TEMPERATURE] = data[TEMPERATURE];

  switch (data[MODE] & MODE_MASK) {
    case AC_MODE_OFF: mode = climate::CLIMATE_MODE_OFF; break;
    case AC_MODE_AUTO: mode = climate::CLIMATE_MODE_AUTO; break;
    case AC_MODE_COOL: mode = climate::CLIMATE_MODE_COOL; break;
    case AC_MODE_DRY: mode = climate::CLIMATE_MODE_DRY; break;
    case AC_MODE_FANONLY: mode = climate::CLIMATE_MODE_FAN_ONLY; break;
    case AC_MODE_HEAT: mode = climate::CLIMATE_MODE_HEAT; break;
  }

  switch (data[MODE] & FAN_MASK) {
    case AC_FAN_AUTO: fan_mode = climate::CLIMATE_FAN_AUTO; break;
    case AC_FAN_LOW: fan_mode = climate::CLIMATE_FAN_LOW; break;
    case AC_FAN_MEDIUM: fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
    case AC_FAN_HIGH: fan_mode = climate::CLIMATE_FAN_HIGH; break;
  }

  preset = (data[10] == 7 || data[10] == 15) ? climate::CLIMATE_PRESET_BOOST : climate::CLIMATE_PRESET_NONE;

  has_valid_state_ = true;
  publish_state();
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
      default: break;
    }
  }

  if (call.get_fan_mode().has_value()) {
    switch (call.get_fan_mode().value()) {
      case climate::CLIMATE_FAN_AUTO: new_fan_speed = AC_FAN_AUTO; break;
      case climate::CLIMATE_FAN_LOW: new_fan_speed = AC_FAN_LOW; break;
      case climate::CLIMATE_FAN_MEDIUM: new_fan_speed = AC_FAN_MEDIUM; break;
      case climate::CLIMATE_FAN_HIGH: new_fan_speed = AC_FAN_HIGH; break;
      default: break;
    }
  }

  if (new_mode == AC_MODE_DRY && new_fan_speed != AC_FAN_LOW) new_fan_speed = AC_FAN_LOW;

  // управление температурой
  if (call.get_target_temperature().has_value()) {
    int temp = call.get_target_temperature().value();
    if (temp >= MIN_VALID_TEMPERATURE && temp <= MAX_VALID_TEMPERATURE)
      data_write_[TEMPERATURE] = (temp - MIN_VALID_TEMPERATURE) * 16;
  }

  // встроенные флаги
  if (display_light_state_) data_write_[10] |= 0x02; else data_write_[10] &= ~0x02;
  if (silent_mode_)      data_write_[11] |= 0x01; else data_write_[11] &= ~0x01;

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
  for (int i = 0; i < size; i++) {
    pstr += sprintf(pstr, "%02X ", message[i]);
  }
  ESP_LOGV(TAG, "%s", str);
}

uint8_t GreeClimate::get_checksum_(const uint8_t *message, size_t size) {
  uint8_t sum = 0;
  for (int i = 2; i < size-1; i++) sum += message[i];
  return sum % 256;
}

void GreeClimate::set_display_light(bool state) {
  display_light_state_ = state;
  ESP_LOGI(TAG, "Display Light set to: %s", state ? "ON" : "OFF");
  control(make_call());
}

void GreeClimate::set_silent_mode(bool state) {
  silent_mode_ = state;
  ESP_LOGI(TAG, "Silent Mode set to: %s", state ? "ON" : "OFF");
  control(make_call());
}

}  // namespace gree
}  // namespace esphome
