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

static const uint8_t CRC_WRITE = 46;

static const uint8_t TEMPERATURE = 9;
static const uint8_t INDOOR_TEMPERATURE = 46;

// component settings
static const uint8_t MIN_VALID_TEMPERATURE = 16;
static const uint8_t MAX_VALID_TEMPERATURE = 30;
static const uint8_t TEMPERATURE_STEP = 1;

// prints user configuration
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
      this->read(); // читаем байт "в никуда"
      continue;
    }

    this->read_array(this->data_read_, sizeof(gree_start_bytes_t));
    receiving_packet_ = (raw_packet->header.start_bytes.u8x2[1] == GREE_START_BYTE);
    
    if (receiving_packet_) {
      this->read_byte( &raw_packet->header.data_length );

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

  // --- ПОДДЕРЖКА SWING ЧЕРЕЗ PRESETS ---
  traits.set_supports_current_temperature(true);
  traits.set_supports_two_point_target_temperature(false);

  traits.set_supported_presets({
    climate::CLIMATE_PRESET_NONE,    // Swing OFF
    climate::CLIMATE_PRESET_COMFORT, // Swing Full
    climate::CLIMATE_PRESET_HOME,    // Swing Top
    climate::CLIMATE_PRESET_AWAY,    // Swing Mid
    climate::CLIMATE_PRESET_BOOST    // Swing Low
  });
  
  return traits;
}

void GreeClimate::read_state_(const uint8_t *data, uint8_t size) {
  // get checksum byte from received data (using the last byte)
  uint8_t data_crc = data[size-1];
  // get checksum byte based on received data (calculating)
  uint8_t get_crc = get_checksum_(data, size);

  if (data_crc != get_crc) {
    ESP_LOGW(TAG, "Invalid checksum.");
    return;
  }

// now we are using only packets with 0x31 as first data byte
  if (data[3] != 49) {
    ESP_LOGW(TAG, "Invalid packet type.");
    return;
  }

  this->target_temperature = data[TEMPERATURE] / 16 + MIN_VALID_TEMPERATURE;
  this->current_temperature = data[INDOOR_TEMPERATURE] - 40; // Используем T-40

  // partially saving current state to previous request
  data_write_[MODE] = data[MODE];
  data_write_[TEMPERATURE] = data[TEMPERATURE];

  // --- ЧТЕНИЕ НОВЫХ СОСТОЯНИЙ ---
  uint8_t byte10 = data[10];
  uint8_t byte13 = data[13];

  // SWING PRESET (Байт 13)
  this->current_swing_value_ = byte13;
  switch (this->current_swing_value_) {
    case SWING_OFF:
      this->preset = climate::CLIMATE_PRESET_NONE;
      break;
    case SWING_FULL:
      this->preset = climate::CLIMATE_PRESET_COMFORT;
      break;
    case SWING_TOP:
      this->preset = climate::CLIMATE_PRESET_HOME;
      break;
    case SWING_MID:
      this->preset = climate::CLIMATE_PRESET_AWAY;
      break;
    case SWING_LOW:
      this->preset = climate::CLIMATE_PRESET_BOOST;
      break;
    default:
      this->preset = climate::CLIMATE_PRESET_NONE;
  }
  
  // TURBO (Байт 10) - 7 или 15
  this->turbo_state_ = (byte10 == TURBO_BIT_COOL || byte10 == TURBO_BIT_HEAT); 
  if (this->turbo_switch_ != nullptr) this->turbo_switch_->publish_state(this->turbo_state_);
  
  // DISPLAY (Байт 10) - Проверяем бит 0x02, который включает/выключает дисплей
  this->display_state_ = (byte10 & 0x02) != 0; 
  if (this->display_switch_ != nullptr) this->display_switch_->publish_state(this->display_state_);

  // update CLIMATE state according AC response
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
      ESP_LOGW(TAG, "Unknown AC MODE: %02X", data[MODE]);
  }

  // get current AC FAN SPEED from its response
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
      ESP_LOGW(TAG, "Unknown AC FAN: %02X", data[MODE]);
  }
  
  this->publish_state();
}

void GreeClimate::control(const climate::ClimateCall &call) {
  data_write_[FORCE_UPDATE] = 175;
  // Байт 13 будет переопределен далее, но оставим 0x20 для отображения температуры
  data_write_[13] = 0x20; 
  
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
        ESP_LOGW(TAG, "Setting of unsupported MODE: %s", call.get_mode().value());
        break;
    }
  }

  // set fan speed only if MODE != DRY (only LOW available)
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
        ESP_LOGW(TAG, "Setting of unsupported FANSPEED: %s", call.get_fan_mode().value());
        break;
    }
  }
  
  // set low speed when DRY mode because other speeds are not available
  if (new_mode == AC_MODE_DRY && new_fan_speed != AC_FAN_LOW) {
    new_fan_speed = AC_FAN_LOW;
  }

  // --- УПРАВЛЕНИЕ SWING PRESET ---
  if (call.get_preset().has_value()) {
    switch (call.get_preset().value()) {
      case climate::CLIMATE_PRESET_NONE:
        this->current_swing_value_ = SWING_OFF;
        break;
      case climate::CLIMATE_PRESET_COMFORT:
        this->current_swing_value_ = SWING_FULL;
        break;
      case climate::CLIMATE_PRESET_HOME:
        this->current_swing_value_ = SWING_TOP;
        break;
      case climate::CLIMATE_PRESET_AWAY:
        this->current_swing_value_ = SWING_MID;
        break;
      case climate::CLIMATE_PRESET_BOOST:
        this->current_swing_value_ = SWING_LOW;
        break;
      default:
        break;
    }
  }

  if (call.get_target_temperature().has_value()) {
    // check if temperature set in valid limits
    if (call.get_target_temperature().value() >= MIN_VALID_TEMPERATURE && call.get_target_temperature().value() <= MAX_VALID_TEMPERATURE)
      data_write_[TEMPERATURE] = (call.get_target_temperature().value() - MIN_VALID_TEMPERATURE) * 16;
  }

  // --- ПРИМЕНЕНИЕ SWING И TURBO/DISPLAY ---
  
  // 1. Применяем SWING (Байт 13)
  data_write_[13] = this->current_swing_value_;
  
  // 2. Применяем TURBO/DISPLAY (Байт 10)
  uint8_t byte10;
  
  if (new_mode == AC_MODE_COOL) {
    byte10 = this->display_state_ ? DISPLAY_ON_MASK : DISPLAY_OFF_MASK;
    if (this->turbo_state_) {
      byte10 = TURBO_BIT_COOL;
    }
  } else if (new_mode == AC_MODE_HEAT) {
    // В режиме HEAT обычно +8 к базовым маскам
    byte10 = (this->display_state_ ? DISPLAY_ON_MASK : DISPLAY_OFF_MASK) + HEAT_MODE_OFFSET;
    if (this->turbo_state_) {
      byte10 = TURBO_BIT_HEAT;
    }
  } else {
    // Для остальных режимов (AUTO/FANONLY/DRY) используем базовую маску
    // Принудительно отключаем Turbo, если режим не COOL/HEAT
    this->turbo_state_ = false; 
    if (this->turbo_switch_ != nullptr) this->turbo_switch_->publish_state(false);
    
    // Используем базовую маску, которая не включает TURBO
    byte10 = this->display_state_ ? DISPLAY_ON_MASK : DISPLAY_OFF_MASK;
  }
  
  data_write_[10] = byte10;

  // 3. Применяем MODE + FAN_SPEED
  data_write_[MODE] = new_mode + new_fan_speed;

  // compute checksum & send data
  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  send_data_(data_write_, sizeof(data_write_));

  // change of force_update byte to "passive" state
  data_write_[FORCE_UPDATE] = 0;
}

// --- Новые функции для управления переключателями (РЕАЛИЗАЦИЯ) ---

void GreeClimate::set_display(bool state) {
  this->display_state_ = state;
  this->control(this->make_call());
}

void GreeClimate::set_turbo(bool state) {
  this->turbo_state_ = state;
  this->control(this->make_call());
}

// --- Вспомогательные функции ---

void GreeClimate::send_data_(const uint8_t *message, uint8_t size) {
  this->write_array(message, size);
  dump_message_("Sent message", message, size);
}

void GreeClimate::dump_message_(const char *title, const uint8_t *message, uint8_t size) {
  ESP_LOGV(TAG, "%s:", title);
  // Используйте ASCII, как вы просили.
  char str[250] = {0};
  char *pstr = str;
  if (size * 3 > sizeof(str)) ESP_LOGE(TAG, "too long byte data");
  for (int i = 0; i < size; i++) {
    pstr += sprintf(pstr, "%02X ", message[i]);
  }
  ESP_LOGV(TAG, "%s", str);
}

uint8_t GreeClimate::get_checksum_(const uint8_t *message, size_t size) {
  // position of crc in packet
  uint8_t position = size - 1;
  uint8_t sum = 0;
  // ignore first 2 bytes & last one
  for (int i = 2; i < position; i++)
    sum += message[i];
  uint8_t crc = sum % 256;
  return crc;
}


}  // namespace gree
}  // namespace esphome
