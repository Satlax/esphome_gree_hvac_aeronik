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

  traits.add_supported_preset(climate::CLIMATE_PRESET_NONE);
  traits.add_supported_preset(climate::CLIMATE_PRESET_BOOST);  // Используем для Display
  return traits;
}

void GreeClimate::read_state_(const uint8_t *data, uint8_t size) {
  uint8_t data_crc = data[size-1];
  uint8_t get_crc = get_checksum_(data, size);
  if (data_crc != get_crc) return;

  // Состояния
  display_state_ = (data[10] & 0x02) ? DISPLAY_ON : DISPLAY_OFF;
  sound_state_ = (data[11] & 0x01) ? SOUND_OFF : SOUND_ON;
  turbo_state_ = (data[10] == 7 || data[10] == 15) ? TURBO_ON : TURBO_OFF;

  // Температуры
  this->target_temperature = data[TEMPERATURE] / 16 + MIN_VALID_TEMPERATURE;
  this->current_temperature = data[INDOOR_TEMPERATURE] - 40;

  data_write_[MODE] = data[MODE];
  data_write_[TEMPERATURE] = data[TEMPERATURE];

  // Режим
  switch (data[MODE] & MODE_MASK) {
    case AC_MODE_OFF: this->mode = climate::CLIMATE_MODE_OFF; break;
    case AC_MODE_AUTO: this->mode = climate::CLIMATE_MODE_AUTO; break;
    case AC_MODE_COOL: this->mode = climate::CLIMATE_MODE_COOL; break;
    case AC_MODE_DRY: this->mode = climate::CLIMATE_MODE_DRY; break;
    case AC_MODE_FANONLY: this->mode = climate::CLIMATE_MODE_FAN_ONLY; break;
    case AC_MODE_HEAT: this->mode = climate::CLIMATE_MODE_HEAT; break;
  }

  switch (data[MODE] & FAN_MASK) {
    case AC_FAN_AUTO: this->fan_mode = climate::CLIMATE_FAN_AUTO; break;
    case AC_FAN_LOW: this->fan_mode = climate::CLIMATE_FAN_LOW; break;
    case
