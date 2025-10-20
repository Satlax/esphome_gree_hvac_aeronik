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

  // Добавляем кастомные fan modes для дополнительных функций
  traits.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH
  });
  traits.add_supported_fan_mode("TURBO");  // Турбо режим как fan mode

  traits.set_supports_current_temperature(true);

  // Добавляем кастомные пресеты для звука и дисплея
  traits.set_supported_presets({
    climate::CLIMATE_PRESET_NONE,
    climate::CLIMATE_PRESET_ECO,      // Используем для звука (ECO = звук выключен)
    climate::CLIMATE_PRESET_COMFORT   // Используем для дисплея (COMFORT = дисплей включен)
  });

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

  // Читаем состояние дисплея (бит 1 в байте 10)
  display_state_ = (data[10] & 0x02) ? DISPLAY_ON : DISPLAY_OFF;
  
  // Читаем состояние звука (бит 0 в байте 11)
  sound_state_ = (data[11] & 0x01) ? SOUND_OFF : SOUND_ON;
  
  // Читаем состояние турбо режима (определяем по специальным кодам)
  turbo_state_ = (data[10] == 7 || data[10] == 15) ? TURBO_ON : TURBO_OFF;

  // Обновляем температуру
  if (data[TEMPERATURE] >= 0 && data[TEMPERATURE] <= 224) { // 0-224 соответствует 16-30°C
    this->target_temperature = data[TEMPERATURE] / 16 + MIN_VALID_TEMPERATURE;
  }
  
  if (data[INDOOR_TEMPERATURE] >= 40 && data[INDOOR_TEMPERATURE] <= 70) { // 40-70 соответствует 0-30°C
    this->current_temperature = data[INDOOR_TEMPERATURE] - 40;
  }

  // Сохраняем текущие настройки режима и температуры для отправки
  data_write_[MODE] = data[MODE];
  data_write_[TEMPERATURE] = data[TEMPERATURE];

  // Обновляем режим работы
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
  }

  // Обновляем скорость вентилятора
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
  }

  // Устанавливаем кастомные режимы
  if (turbo_state_ == TURBO_ON) {
    this->fan_mode = "TURBO";  // Турбо режим как fan mode
  }

  // Устанавливаем пресеты для звука и дисплея
  if (sound_state_ == SOUND_OFF) {
    this->preset = climate::CLIMATE_PRESET_ECO;  // ECO = звук выключен
  } else if (display_state_ == DISPLAY_ON) {
    this->preset = climate::CLIMATE_PRESET_COMFORT;  // COMFORT = дисплей включен
  } else {
    this->preset = climate::CLIMATE_PRESET_NONE;
  }

  has_valid_state_ = true;
  this->publish_state();
}

void GreeClimate::control(const climate::ClimateCall &call) {
  // Устанавливаем флаг принудительного обновления
  data_write_[FORCE_UPDATE] = 175;

  uint8_t new_mode = data_write_[MODE] & MODE_MASK;
  uint8_t new_fan_speed = data_write_[MODE] & FAN_MASK;

  // Обработка изменения режима работы
  if (call.get_mode().has_value()) {
    climate::ClimateMode esp_mode = call.get_mode().value();
    switch (esp_mode) {
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
        new_fan_speed = AC_FAN_LOW; // В режиме осушения всегда низкая скорость
        break;
      case climate::CLIMATE_MODE_FAN_ONLY: 
        new_mode = AC_MODE_FANONLY; 
        break;
      case climate::CLIMATE_MODE_HEAT: 
        new_mode = AC_MODE_HEAT; 
        break;
      default: 
        break;
    }
  }

  // Обработка изменения скорости вентилятора и турбо режима
  if (call.get_fan_mode().has_value()) {
    auto fan_mode_value = call.get_fan_mode().value();
    
    if (fan_mode_value == "TURBO") {
      // Включаем турбо режим
      turbo_state_ = TURBO_ON;
      new_fan_speed = AC_FAN_HIGH; // В турбо режиме обычно высокая скорость
    } else {
      // Обычные режимы вентилятора
      turbo_state_ = TURBO_OFF;
      switch (fan_mode_value) {
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
          break;
      }
    }
  }

  // Обработка пресетов для звука и дисплея
  if (call.get_preset().has_value()) {
    auto preset_value = call.get_preset().value();
    
    if (preset_value == climate::CLIMATE_PRESET_ECO) {
      // ECO = звук выключен (тихий режим)
      sound_state_ = SOUND_OFF;
      display_state_ = DISPLAY_OFF;
    } else if (preset_value == climate::CLIMATE_PRESET_COMFORT) {
      // COMFORT = дисплей включен
      display_state_ = DISPLAY_ON;
      sound_state_ = SOUND_ON;
    } else if (preset_value == climate::CLIMATE_PRESET_NONE) {
      // NONE = все выключено
      display_state_ = DISPLAY_OFF;
      sound_state_ = SOUND_ON;
    }
  }

  // В режиме осушения всегда устанавливаем низкую скорость
  if (new_mode == AC_MODE_DRY) {
    new_fan_speed = AC_FAN_LOW;
  }

  // Обработка изменения температуры
  if (call.get_target_temperature().has_value()) {
    float temp = call.get_target_temperature().value();
    if (temp >= MIN_VALID_TEMPERATURE && temp <= MAX_VALID_TEMPERATURE) {
      data_write_[TEMPERATURE] = (uint8_t)((temp - MIN_VALID_TEMPERATURE) * 16);
    }
  }

  // Применяем основные настройки режима и вентилятора
  data_write_[MODE] = new_mode | new_fan_speed;
  
  // Устанавливаем флаги дисплея и звука
  data_write_[10] = (data_write_[10] & ~0x02) | (display_state_ & 0x02);
  data_write_[11] = (data_write_[11] & ~0x01) | (sound_state_ & 0x01);

  // Если включен турбо режим, переопределяем настройки
  if (turbo_state_ == TURBO_ON) {
    data_write_[10] = TURBO_ON;
  }

  // Рассчитываем и устанавливаем контрольную сумму
  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  
  // Отправляем данные
  send_data_(data_write_, sizeof(data_write_));

  // Сбрасываем флаг принудительного обновления
  data_write_[FORCE_UPDATE] = 0;
  
  // Обновляем состояние
  this->publish_state();
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
  for (size_t i = 2; i < size - 1; i++) {
    sum += message[i];
  }
  return sum % 256;
}

}  // namespace gree
}  // namespace esphome
