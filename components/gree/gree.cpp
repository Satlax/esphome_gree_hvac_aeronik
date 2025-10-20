#include "gree.h"
#include "esphome/core/log.h"

namespace esphome {
namespace gree {

static const char *const TAG = "gree";

// byte positions and masks
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
      this->read(); // discard byte
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
  // do not send until we have a valid state read from AC
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

  // Use existing presets but interpret them as our three toggles:
  // BOOST  -> Turbo toggle
  // COMFORT -> Display toggle
  // SLEEP  -> Sound toggle
  traits.set_supported_presets({
    climate::CLIMATE_PRESET_NONE,
    climate::CLIMATE_PRESET_BOOST,
    climate::CLIMATE_PRESET_COMFORT,
    climate::CLIMATE_PRESET_SLEEP
  });

  return traits;
}

void GreeClimate::read_state_(const uint8_t *data, uint8_t size) {
  // validate checksum
  uint8_t data_crc = data[size - 1];
  uint8_t calc_crc = get_checksum_(data, size);
  if (data_crc != calc_crc) {
    ESP_LOGW(TAG, "Invalid checksum: got %02X calc %02X", data_crc, calc_crc);
    return;
  }

  if (data[3] != 49) {
    ESP_LOGW(TAG, "Invalid packet type: %d", data[3]);
    return;
  }

  // parse display bit (byte index 10, bit 1)
  parsed_display_ = (data[10] & 0x02) ? DISPLAY_ON : DISPLAY_OFF;
  display_enabled_ = (parsed_display_ == DISPLAY_ON);

  // parse sound bit (byte index 11, bit 0) -- note: in device, 1 may mean sound off
  parsed_sound_ = (data[11] & 0x01) ? SOUND_OFF : SOUND_ON;
  sound_enabled_ = (parsed_sound_ == SOUND_ON);

  // parse turbo: commonly byte10 == 7 or 15 indicates turbo active
  parsed_turbo_ = (data[10] == 7 || data[10] == 15) ? TURBO_ON_CODE : TURBO_OFF;
  turbo_enabled_ = (parsed_turbo_ == TURBO_ON_CODE);

  // temperatures
  this->target_temperature = data[TEMPERATURE] / 16 + MIN_VALID_TEMPERATURE;
  this->current_temperature = data[INDOOR_TEMPERATURE] - 40;

  // save current mode / temp values to be able to send later
  data_write_[MODE] = data[MODE];
  data_write_[TEMPERATURE] = data[TEMPERATURE];

  // update modes/fan fields
  switch (data[MODE] & MODE_MASK) {
    case AC_MODE_OFF: this->mode = climate::CLIMATE_MODE_OFF; break;
    case AC_MODE_AUTO: this->mode = climate::CLIMATE_MODE_AUTO; break;
    case AC_MODE_COOL: this->mode = climate::CLIMATE_MODE_COOL; break;
    case AC_MODE_DRY: this->mode = climate::CLIMATE_MODE_DRY; break;
    case AC_MODE_FANONLY: this->mode = climate::CLIMATE_MODE_FAN_ONLY; break;
    case AC_MODE_HEAT: this->mode = climate::CLIMATE_MODE_HEAT; break;
    default: break;
  }

  switch (data[MODE] & FAN_MASK) {
    case AC_FAN_AUTO: this->fan_mode = climate::CLIMATE_FAN_AUTO; break;
    case AC_FAN_LOW: this->fan_mode = climate::CLIMATE_FAN_LOW; break;
    case AC_FAN_MEDIUM: this->fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
    case AC_FAN_HIGH: this->fan_mode = climate::CLIMATE_FAN_HIGH; break;
    default: break;
  }

  // set preset for UI briefly — we choose NONE so UI won't "lock" into a single preset,
  // but we log current internal states for debugging.
  this->preset = climate::CLIMATE_PRESET_NONE;

  ESP_LOGI(TAG, "Parsed state: turbo=%s display=%s sound=%s tgt=%d cur=%d",
           turbo_enabled_ ? "ON":"OFF",
           display_enabled_ ? "ON":"OFF",
           sound_enabled_ ? "ON":"OFF",
           (int)this->target_temperature, (int)this->current_temperature);

  has_valid_state_ = true;
  publish_state();
}

void GreeClimate::control(const climate::ClimateCall &call) {
  // set force update
  data_write_[FORCE_UPDATE] = 175;

  // mode & fan handling (existing logic)
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

  // target temperature
  if (call.get_target_temperature().has_value()) {
    float t = call.get_target_temperature().value();
    if (t >= MIN_VALID_TEMPERATURE && t <= MAX_VALID_TEMPERATURE) {
      data_write_[TEMPERATURE] = (uint8_t)((t - MIN_VALID_TEMPERATURE) * 16);
    }
  }

  // --- HANDLE PRESETS AS TOGGLES ---
  if (call.get_preset().has_value()) {
    auto p = call.get_preset().value();
    switch (p) {
      case climate::CLIMATE_PRESET_BOOST:
        // toggle turbo
        turbo_enabled_ = !turbo_enabled_;
        ESP_LOGI(TAG, "Preset BOOST received -> toggling turbo to %s", turbo_enabled_ ? "ON":"OFF");
        break;

      case climate::CLIMATE_PRESET_COMFORT:
        // toggle display
        display_enabled_ = !display_enabled_;
        ESP_LOGI(TAG, "Preset COMFORT received -> toggling display to %s", display_enabled_ ? "ON":"OFF");
        break;

      case climate::CLIMATE_PRESET_SLEEP:
        // toggle sound (silent)
        sound_enabled_ = !sound_enabled_;
        ESP_LOGI(TAG, "Preset SLEEP received -> toggling sound to %s", sound_enabled_ ? "ON":"OFF");
        break;

      case climate::CLIMATE_PRESET_NONE:
      default:
        // nothing
        break;
    }
    // after processing preset toggle we do not leave preset active (so user can press multiple)
    this->preset = climate::CLIMATE_PRESET_NONE;
  }

  // apply display bit (byte 10, bit 1)
  if (display_enabled_) data_write_[10] |= 0x02; else data_write_[10] &= ~0x02;

  // apply sound bit (byte 11, bit 0) -- note: device may use 1 meaning SOUND_OFF
  if (sound_enabled_) data_write_[11] &= ~0x01; else data_write_[11] |= 0x01;

  // apply turbo: if turbo_enabled_ set turbo code in byte 10 (device specific)
  if (turbo_enabled_) {
    // set turbo code (7) in byte 10 — careful: this overrides some bits, but original code did same
    data_write_[10] = TURBO_ON_CODE;
  } else {
    // ensure normal (remove turbo code); keep other bits (display) consistent
    // restore display bit if needed (we already set above)
    // clear turbo code effects: we won't forcibly zero whole byte, we keep previous but clear known turbo values
    if (data_write_[10] == TURBO_ON_CODE || data_write_[10] == 15) {
      data_write_[10] = 0x00;
      if (display_enabled_) data_write_[10] |= 0x02;
    }
  }

  // combine mode & fan
  data_write_[MODE] = new_mode | (new_fan_speed & FAN_MASK);

  // checksum & send
  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  send_data_(data_write_, sizeof(data_write_));

  // clear force update flag
  data_write_[FORCE_UPDATE] = 0;

  // publish current state so UI updates (we still show preset NONE)
  publish_state();
}

void GreeClimate::send_data_(const uint8_t *message, uint8_t size) {
  this->write_array(message, size);
  dump_message_("Sent message", message, size);
}

void GreeClimate::dump_message_(const char *title, const uint8_t *message, uint8_t size) {
  ESP_LOGV(TAG, "%s:", title);
  char str[256] = {0};
  char *p = str;
  for (uint8_t i = 0; i < size; i++) {
    p += sprintf(p, "%02X ", message[i]);
  }
  ESP_LOGV(TAG, "%s", str);
}

uint8_t GreeClimate::get_checksum_(const uint8_t *message, size_t size) {
  uint8_t pos = (uint8_t)(size - 1);
  uint8_t sum = 0;
  for (size_t i = 2; i < pos; i++) sum += message[i];
  return sum % 256;
}

}  // namespace gree
}  // namespace esphome
