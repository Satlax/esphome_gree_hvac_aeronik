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

  // ИСПРАВЛЕНО: Добавлен yield() для предотвращения Soft WDT
  while (!receiving_packet_ && this->available() >= sizeof(gree_header_t)) {
    if (this->peek() != GREE_START_BYTE) {
      this->read();
      yield(); // <-- ДОБАВЛЕНО: Даем процессору передышку
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
    
    yield(); // <-- ДОБАВЛЕНО: Еще одна передышка после обработки заголовка
  }

  if (receiving_packet_ && this->available() >= raw_packet->header.data_length) {
    this->read_array(raw_packet->data, raw_packet->header.data_length);

    uint8_t total_size = raw_packet->header.data_length + sizeof(gree_header_t);
    
    if (total_size > 0 && total_size <= GREE_RX_BUFFER_SIZE) {
      dump_message_("Read array", this->data_read_, total_size);
      read_state_(this->data_read_, total_size);
    } else {
      ESP_LOGW(TAG, "Invalid total packet size: %d", total_size);
    }

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

  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);

  for (auto preset : this->supported_presets_) {
    traits.add_supported_preset(preset);
  }
  traits.add_supported_preset(climate::CLIMATE_PRESET_NONE);

  return traits;
}

void GreeClimate::read_state_(const uint8_t *data, uint8_t size) {
  if (size < 3) {
    ESP_LOGW(TAG, "Packet too small for processing: %d", size);
    return;
  }

  uint8_t data_crc = data[size-1];
  uint8_t get_crc = get_checksum_(data, size);

  if (data_crc != get_crc) {
    ESP_LOGW(TAG, "Invalid checksum.");
    return;
  }

  if (data[2] != 47 && data[2] != 49) {
    ESP_LOGW(TAG, "Invalid packet length: %d", data[2]);
    return;
  }

  this->target_temperature = data[9] / 16 + MIN_VALID_TEMPERATURE;
  this->current_temperature = data[46] - 40;

  data_write_[8] = data[8];
  data_write_[9] = data[9];
  data_write_[10] = data[10];
  data_write_[12] = data[12];
  data_write_[13] = data[13];

  switch (data[8] & 0b11110000) {
    case 0x10: 
    case 0x00: 
      this->mode = climate::CLIMATE_MODE_OFF; break;
    case 0x80: this->mode = climate::CLIMATE_MODE_AUTO; break;
    case 0x90: this->mode = climate::CLIMATE_MODE_COOL; break;
    case 0xA0: this->mode = climate::CLIMATE_MODE_DRY; break;
    case 0xB0: this->mode = climate::CLIMATE_MODE_FAN_ONLY; break;
    case 0xC0: this->mode = climate::CLIMATE_MODE_HEAT; break;
    default: ESP_LOGW(TAG, "Unknown AC MODE: %02X", data[8]);
  }

  switch (data[8] & 0b00001111) {
    case 0x00: this->fan_mode = climate::CLIMATE_FAN_AUTO; break;
    case 0x01: this->fan_mode = climate::CLIMATE_FAN_LOW; break;
    case 0x02: this->fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
    case 0x03: this->fan_mode = climate::CLIMATE_FAN_HIGH; break;
    default: ESP_LOGW(TAG, "Unknown AC fan: %02X", data[8]);
  }

  if (data[10] == 7 || data[10] == 15) {
    this->turbo_state_ = true;
  } else {
    this->turbo_state_ = false;
  }

  this->display_state_ = (data[10] & 0x02) != 0;

  this->publish_state();
}

void GreeClimate::control(const climate::ClimateCall &call) {
  data_write_[7] = 175;

  uint8_t new_mode = data_write_[8] & 0b11110000;
  uint8_t new_fan_speed = data_write_[8] & 0b00001111;

  if (call.get_mode().has_value()) {
    switch (call.get_mode().value()) {
      case climate::CLIMATE_MODE_OFF: new_mode = 0x10; break;
      case climate::CLIMATE_MODE_AUTO: new_mode = 0x80; break;
      case climate::CLIMATE_MODE_COOL: new_mode = 0x90; break;
      case climate::CLIMATE_MODE_DRY: 
        new_mode = 0xA0;
        new_fan_speed = 0x01;
        break;
      case climate::CLIMATE_MODE_FAN_ONLY: new_mode = 0xB0; break;
      case climate::CLIMATE_MODE_HEAT: new_mode = 0xC0; break;
      default: ESP_LOGW(TAG, "Setting of unsupported MODE: %d", (int)call.get_mode().value()); break;
    }
  }

  if (call.get_fan_mode().has_value()) {
    switch (call.get_fan_mode().value()) {
      case climate::CLIMATE_FAN_AUTO: new_fan_speed = 0x00; break;
      case climate::CLIMATE_FAN_LOW: new_fan_speed = 0x01; break;
      case climate::CLIMATE_FAN_MEDIUM: new_fan_speed = 0x02; break;
      case climate::CLIMATE_FAN_HIGH: new_fan_speed = 0x03; break;
      default: ESP_LOGW(TAG, "Setting of unsupported FANSPEED: %d", (int)call.get_fan_mode().value()); break;
    }
  }

  if (new_mode == 0xA0 && new_fan_speed != 0x01) {
    new_fan_speed = 0x01;
  }

  if (call.get_target_temperature().has_value()) {
    if (call.get_target_temperature().value() >= MIN_VALID_TEMPERATURE && call.get_target_temperature().value() <= MAX_VALID_TEMPERATURE)
      data_write_[9] = (call.get_target_temperature().value() - MIN_VALID_TEMPERATURE) * 16;
  }

  data_write_[8] = new_mode + new_fan_speed;

  data_write_[46] = get_checksum_(data_write_, sizeof(data_write_));
  send_data_(data_write_, sizeof(data_write_));

  data_write_[7] = 0;
}

void GreeClimate::send_data_(const uint8_t *message, uint8_t size) {
  this->write_array(message, size);
  dump_message_("Sent message", message, size);
}

void GreeClimate::dump_message_(const char *title, const uint8_t *message, uint8_t size) {
  if (size == 0 || size > GREE_RX_BUFFER_SIZE) {
    ESP_LOGW(TAG, "Invalid size for dump: %d", size);
    return;
  }

  ESP_LOGV(TAG, "%s:", title);
  char str[250] = {0};
  char *pstr = str;
  
  for (uint8_t i = 0; i < size; i++) {
    pstr += sprintf(pstr, "%02X ", message[i]);
  }
  ESP_LOGV(TAG, "%s", str);
}

uint8_t GreeClimate::get_checksum_(const uint8_t *message, uint8_t size) {
  if (size < 3) {
    return 0;
  }
  
  uint8_t position = size - 1;
  uint8_t sum = 0;
  for (uint8_t i = 2; i < position; i++) {
    sum += message[i];
  }
  return sum % 256;
}

void GreeClimate::set_display(bool state) {
  this->display_state_ = state;
  data_write_[7] = 175;
  
  if (state)
    data_write_[10] = data_write_[10] | 0x02;
  else
    data_write_[10] = data_write_[10] & (~0x02);

  data_write_[46] = get_checksum_(data_write_, sizeof(data_write_));
  send_data_(data_write_, sizeof(data_write_));
  
  data_write_[7] = 0;
}

void GreeClimate::set_turbo(bool state) {
  this->turbo_state_ = state;
  data_write_[7] = 175;
  
  uint8_t mode_only = data_write_[8] & 0b11110000;
  uint8_t base_val = 6;
  if (mode_only == 0xC0) base_val = 14; 
  
  uint8_t target_val = state ? (base_val + 1) : base_val;

  bool display_was_on = (data_write_[10] & 0x02) != 0;
  
  data_write_[10] = target_val;
  if (display_was_on) {
    data_write_[10] |= 0x02;
  }

  data_write_[46] = get_checksum_(data_write_, sizeof(data_write_));
  send_data_(data_write_, sizeof(data_write_));
  
  data_write_[7] = 0;
}

}  // namespace gree
}  // namespace esphome
