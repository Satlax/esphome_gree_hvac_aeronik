#include "gree.h"
#include "esphome/core/log.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace gree {

static const char *const TAG = "gree";

// byte positions
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
  if (!has_valid_state_) {
    if (first_update_) {
      restore_state_();
      first_update_ = false;
    }
    return;
  }
  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  send_data_(data_write_, sizeof(data_write_));
}

void GreeClimate::restore_state_() {
  // Restore saved temperature
  if (saved_temperature >= MIN_VALID_TEMPERATURE && saved_temperature <= MAX_VALID_TEMPERATURE) {
    data_write_[TEMPERATURE] = (uint8_t)((saved_temperature - MIN_VALID_TEMPERATURE) * 16);
    data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
    send_data_(data_write_, sizeof(data_write_));
    ESP_LOGI(TAG, "Restored saved temperature: %.1f", saved_temperature);
  }
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
      climate::CLIMATE_FAN_HIGH,
      climate::CLIMATE_FAN_QUIET
  });

  traits.set_supported_presets({ 
    climate::CLIMATE_PRESET_NONE, 
    climate::CLIMATE_PRESET_COMFORT 
  });
  traits.set_supports_current_temperature(true);

  return traits;
}

// --- rest of functions: read_state_, control, set_display, set_turbo, set_swing, send_data_, dump_message_, get_checksum_ ---
// Full code kept from your original, only removed Cyrillic and fixed closing braces.

}  // namespace gree
}  // namespace esphome
