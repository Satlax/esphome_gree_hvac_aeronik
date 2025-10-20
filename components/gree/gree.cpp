#include "gree.h"
#include "esphome/core/log.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace gree {

static const char *const TAG = "gree";

// byte positions
static const uint8_t FORCE_UPDATE = 7;
static const uint8_t MODE = 8;
static const uint8_t MODE_MASK #include "gree.h"
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

void GreeClimate::read_state_(const uint8_t *data, uint8_t size) {
  uint8_t data_crc = data[size-1];
  if (data_crc != get_checksum_(data, size)) {
    ESP_LOGW(TAG, "Invalid checksum.");
    return;
  }

  if (data[3] != 49) {
    ESP_LOGW(TAG, "Invalid packet type.");
    return;
  }

  // Update states from packet
  // Byte 10: Bit 1 (0x02) - Display, Bit 0 (0x01) - Turbo
  display_state_ = (data[10] & DISPLAY_ON) ? DISPLAY_ON : DISPLAY_OFF;
  turbo_state_ = (data[10] & TURBO_BIT);

  // Swing (Byte 11)
  swing_state_ = (data[11] == SWING_ON) ? SWING_ON : SWING_OFF;
  
  // Update switches
  if (turbo_switch != nullptr) {
    turbo_switch->publish_state(turbo_state_);
  }
  if (swing_switch != nullptr) {
    swing_switch->publish_state(swing_state_ == SWING_ON);
  }

  target_temperature = data[TEMPERATURE] / 16 + MIN_VALID_TEMPERATURE;
  current_temperature = data[INDOOR_TEMPERATURE] - 40;

  // Save current temperature
  saved_temperature = target_temperature;

  data_write_[MODE] = data[MODE];
  data_write_[TEMPERATURE] = data[TEMPERATURE];

  switch (data[MODE] & MODE_MASK) {
    case AC_MODE_OFF: mode = climate::CLIMATE_MODE_OFF; break;
    case AC_MODE_AUTO: mode = climate::CLIMATE_MODE_AUTO; break;
    case AC_MODE_COOL: mode = climate::CLIMATE_MODE_COOL; break;
    case AC_MODE_DRY: mode = climate::CLIMATE_MODE_DRY; break;
    case AC_MODE_FANONLY: mode = climate::CLIMATE_MODE_FAN_ONLY; break;
    case AC_MODE_HEAT: mode = climate::CLIMATE_MODE_HEAT; break;
    default: mode = climate::CLIMATE_MODE_OFF; break;
  }

  switch (data[MODE] & FAN_MASK) {
    case AC_FAN_AUTO: fan_mode = climate::CLIMATE_FAN_AUTO; break;
    case AC_FAN_LOW: fan_mode = climate::CLIMATE_FAN_LOW; break;
    case AC_FAN_MEDIUM: fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
    case AC_FAN_HIGH: fan_mode = climate::CLIMATE_FAN_HIGH; break;
    case AC_FAN_QUIET: fan_mode = climate::CLIMATE_FAN_LOW; break;
    default: fan_mode = climate::CLIMATE_FAN_AUTO; break;
  }

  // Display via preset
  preset = display_state_ == DISPLAY_ON ? climate::CLIMATE_PRESET_COMFORT : climate::CLIMATE_PRESET_NONE;

  has_valid_state_ = true;
  publish_state();
}

void GreeClimate::control(const climate::ClimateCall &call) {
  data_write_[FORCE_UPDATE] = 175;

  uint8_t new_mode = data_write_[MODE] & MODE_MASK;
  uint8_t new_fan_speed = data_write_[MODE] & FAN_MASK;

  if (call.get_mode().has_value()) {
    climate::ClimateMode esp_mode = call.get_mode().value();
    switch (esp_mode) {
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
    climate::ClimateFanMode esp_fan_mode = call.get_fan_mode().value();
    switch (esp_fan_mode) {
      case climate::CLIMATE_FAN_AUTO: new_fan_speed = AC_FAN_AUTO; break;
      case climate::CLIMATE_FAN_LOW: new_fan_speed = AC_FAN_LOW; break;
      case climate::CLIMATE_FAN_MEDIUM: new_fan_speed = AC_FAN_MEDIUM; break;
      case climate::CLIMATE_FAN_HIGH: new_fan_speed = AC_FAN_HIGH; break;
      case climate::CLIMATE_FAN_QUIET: new_fan_speed = AC_FAN_LOW; break;
      default: break;
    }
  }

  // Handle preset for display
  if (call.get_preset().has_value()) {
    auto preset_value = call.get_preset().value();
    if (preset_value == climate::CLIMATE_PRESET_COMFORT) {
      display_state_ = DISPLAY_ON;
    } else {
      display_state_ = DISPLAY_OFF;
    }
  }

  if (call.get_target_temperature().has_value()) {
    float temp = call.get_target_temperature().value();
    if (temp >= MIN_VALID_TEMPERATURE && temp <= MAX_VALID_TEMPERATURE) {
      data_write_[TEMPERATURE] = (uint8_t)((temp - MIN_VALID_TEMPERATURE) * 16);
      // Save temperature
      saved_temperature = temp;
    }
  }

  // Apply main mode and fan speed settings
  data_write_[MODE] = new_mode | new_fan_speed;

  // --- CORRECTED LOGIC FOR BYTE 10 (Turbo and Display) ---

  // Clear bits 0 and 1
  data_write_[10] &= ~0x03; 

  // 1. Set Turbo bit (0x01)
  if (turbo_state_) {
    data_write_[10] |= TURBO_BIT; // 0x01
  }
  
  // 2. Set Display bit (0x02)
  if (display_state_ == DISPLAY_ON) {
    data_write_[10] |= DISPLAY_ON; // 0x02
  }

  // --- Swing Control (Byte 11) ---

  if (swing_state_ == SWING_ON) {
    data_write_[11] = SWING_ON; // 0x10
  } else {
    data_write_[11] = SWING_OFF; // 0x60
  }

  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  send_data_(data_write_, sizeof(data_write_));

  data_write_[FORCE_UPDATE] = 0;
  
  publish_state();
}

// --- Setters for Switch components ---

void GreeClimate::set_display(bool state) {
  display_state_ = state ? DISPLAY_ON : DISPLAY_OFF;
  control(this->make_call());
}

void GreeClimate::set_turbo(bool state) {
  turbo_state_ = state;
  control(this->make_call());
}

void GreeClimate::set_swing(bool state) {
  swing_state_ = state ? SWING_ON : SWING_OFF;
  control(this->make_call());
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
  for (size_t i = 2; i < size - 1; i++) sum += message[i];
  return sum % 256;
}

}  // namespace gree
}  // namespace esphome= 0b11110000;
static const uint8_t FAN_MASK = 0b00001111;
static const uint8_t CRC_WRITE = 46;
static const uint8_t TEMPERATURE = 9;
static const uint8_t INDOOR_TEMPERATURE = 46; 

static const uint8_t MIN_VALID_TEMPERATURE = 16;
static const uint8_t MAX_VALID_TEMPERATURE = 30;
static const uint8_t TEMPERATURE_STEP = 1;

void GreeClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "Gree:");
  ESP_LOGCONFIG(TAG, "  Update interval: %u", this->get_update_interval());
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
  // Восстанавливаем сохраненную температуру
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

void GreeClimate::read_state_(const uint8_t *data, uint8_t size) {
  uint8_t data_crc = data[size-1];
  if (data_crc != get_checksum_(data, size)) {
    ESP_LOGW(TAG, "Invalid checksum.");
    return;
  }

  if (data[3] != 49) {
    ESP_LOGW(TAG, "Invalid packet type.");
    return;
  }

  // Обновляем состояния из пакета
  // Байт 10: Бит 1 (0x02) - Дисплей, Бит 0 (0x01) - Турбо
  display_state_ = (data[10] & DISPLAY_ON) ? DISPLAY_ON : DISPLAY_OFF;
  turbo_state_ = (data[10] & TURBO_BIT);

  // Swing (Байт 11)
  swing_state_ = (data[11] == SWING_ON) ? SWING_ON : SWING_OFF;
  
  // Обновляем переключатели
  if (turbo_switch != nullptr) {
    turbo_switch->publish_state(turbo_state_);
  }
  if (swing_switch != nullptr) {
    swing_switch->publish_state(swing_state_ == SWING_ON);
  }

  target_temperature = data[TEMPERATURE] / 16 + MIN_VALID_TEMPERATURE;
  current_temperature = data[INDOOR_TEMPERATURE] - 40;

  // Сохраняем текущую температуру
  saved_temperature = target_temperature;

  data_write_[MODE] = data[MODE];
  data_write_[TEMPERATURE] = data[TEMPERATURE];

  switch (data[MODE] & MODE_MASK) {
    case AC_MODE_OFF: mode = climate::CLIMATE_MODE_OFF; break;
    case AC_MODE_AUTO: mode = climate::CLIMATE_MODE_AUTO; break;
    case AC_MODE_COOL: mode = climate::CLIMATE_MODE_COOL; break;
    case AC_MODE_DRY: mode = climate::CLIMATE_MODE_DRY; break;
    case AC_MODE_FANONLY: mode = climate::CLIMATE_MODE_FAN_ONLY; break;
    case AC_MODE_HEAT: mode = climate::CLIMATE_MODE_HEAT; break;
    default: mode = climate::CLIMATE_MODE_OFF; break;
  }

  switch (data[MODE] & FAN_MASK) {
    case AC_FAN_AUTO: fan_mode = climate::CLIMATE_FAN_AUTO; break;
    case AC_FAN_LOW: fan_mode = climate::CLIMATE_FAN_LOW; break;
    case AC_FAN_MEDIUM: fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
    case AC_FAN_HIGH: fan_mode = climate::CLIMATE_FAN_HIGH; break;
    default: fan_mode = climate::CLIMATE_FAN_AUTO; break;
  }

  // Дисплей через preset
  preset = display_state_ == DISPLAY_ON ? climate::CLIMATE_PRESET_COMFORT : climate::CLIMATE_PRESET_NONE;

  has_valid_state_ = true;
  publish_state();
}

void GreeClimate::control(const climate::ClimateCall &call) {
  data_write_[FORCE_UPDATE] = 175;

  uint8_t new_mode = data_write_[MODE] & MODE_MASK;
  uint8_t new_fan_speed = data_write_[MODE] & FAN_MASK;

  if (call.get_mode().has_value()) {
    climate::ClimateMode esp_mode = call.get_mode().value();
    switch (esp_mode) {
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
    climate::ClimateFanMode esp_fan_mode = call.get_fan_mode().value();
    switch (esp_fan_mode) {
      case climate::CLIMATE_FAN_AUTO: new_fan_speed = AC_FAN_AUTO; break;
      case climate::CLIMATE_FAN_LOW: new_fan_speed = AC_FAN_LOW; break;
      case climate::CLIMATE_FAN_MEDIUM: new_fan_speed = AC_FAN_MEDIUM; break;
      case climate::CLIMATE_FAN_HIGH: new_fan_speed = AC_FAN_HIGH; break;
      case climate::CLIMATE_FAN_QUIET: new_fan_speed = AC_FAN_LOW; break;
      default: break;
    }
  }

  // Обработка preset для дисплея
  if (call.get_preset().has_value()) {
    auto preset_value = call.get_preset().value();
    if (preset_value == climate::CLIMATE_PRESET_COMFORT) {
      display_state_ = DISPLAY_ON;
    } else {
      display_state_ = DISPLAY_OFF;
    }
  }

  if (call.get_target_temperature().has_value()) {
    float temp = call.get_target_temperature().value();
    if (temp >= MIN_VALID_TEMPERATURE && temp <= MAX_VALID_TEMPERATURE) {
      data_write_[TEMPERATURE] = (uint8_t)((temp - MIN_VALID_TEMPERATURE) * 16);
      // Сохраняем температуру
      saved_temperature = temp;
    }
  }

  // Применяем основные настройки режима и вентилятора
  data_write_[MODE] = new_mode | new_fan_speed;

  // --- ИСПРАВЛЕННАЯ ЛОГИКА ДЛЯ БАЙТА 10 (Турбо и Дисплей) ---

  // Начинаем с базового значения 0x0C (очищаем биты 0 и 1)
  // 0x0C = 0b1100. Это нужно для того, чтобы не сбросить старшие биты
  data_write_[10] &= ~0x03; // Очищаем биты 0 и 1

  // 1. Устанавливаем бит Турбо (0x01)
  if (turbo_state_) {
    data_write_[10] |= TURBO_BIT; // 0x01
  }
  
  // 2. Устанавливаем бит Дисплея (0x02)
  if (display_state_ == DISPLAY_ON) {
    data_write_[10] |= DISPLAY_ON; // 0x02
  }

  // --- Управление Swing (Байт 11) ---

  if (swing_state_ == SWING_ON) {
    data_write_[11] = SWING_ON; // 0x10
  } else {
    data_write_[11] = SWING_OFF; // 0x60
  }

  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  send_data_(data_write_, sizeof(data_write_));

  data_write_[FORCE_UPDATE] = 0;
  
  publish_state();
}

// --- Сеттеры для Switch-компонентов ---

void GreeClimate::set_display(bool state) {
  display_state_ = state ? DISPLAY_ON : DISPLAY_OFF;
  control(this->make_call());
}

void GreeClimate::set_turbo(bool state) {
  turbo_state_ = state;
  control(this->make_call());
}

void GreeClimate::set_swing(bool state) {
  swing_state_ = state ? SWING_ON : SWING_OFF;
  control(this->make_call());
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
  for (size_t i = 2; i < size - 1; i++) sum += message[i];
  return sum % 256;
}

}  // namespace gree
}  // namespace esphome
