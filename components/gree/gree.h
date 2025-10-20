#pragma once

#include "esphome.h"

namespace esphome {
namespace gree {

class GreeClimate : public climate::Climate, public Component {
 public:
  void setup() override {
    // Инициализация
  }

  void control(const climate::ClimateCall &call) override {
    if (call.get_mode().has_value()) {
      // Управление режимом
    }
    if (call.get_fan_mode().has_value()) {
      // Управление скоростью вентилятора
    }
    if (call.get_preset().has_value()) {
      // Управление пресетами
      handle_preset(call.get_preset().value());
    }
  }

  void handle_preset(climate::ClimatePreset preset) {
    switch (preset) {
      case climate::CLIMATE_PRESET_BOOST:
        // Включение Turbo Mode
        break;
      case climate::CLIMATE_PRESET_SLEEP:
        // Включение Silent Mode
        break;
      case climate::CLIMATE_PRESET_CUSTOM_1:
        // Включение Display Light
        break;
      default:
        break;
    }
  }

  climate::ClimateTraits traits() override {
    auto traits = climate::ClimateTraits();
    traits.set_supports_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_COOL});
    traits.set_supports_presets({
      climate::CLIMATE_PRESET_BOOST,
      climate::CLIMATE_PRESET_SLEEP,
      climate::CLIMATE_PRESET_CUSTOM_1,
    });
    return traits;
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Gree Climate:");
    // Вывод конфигурации
  }

 private:
  static const char *TAG;
};

}  // namespace gree
}  // namespace esphome
