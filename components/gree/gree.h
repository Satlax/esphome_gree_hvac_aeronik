#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/log.h"

namespace esphome {
namespace gree {

// ... (все enum и определения без изменений) ...

class GreeClimate : public climate::Climate, public uart::UARTDevice, public PollingComponent {
 public:
  void loop() override;
  void update() override;
  void dump_config() override;
  void control(const climate::ClimateCall &call) override;

  void set_supported_presets(const std::vector<climate::ClimatePreset> &presets) {
    supported_presets_.clear();
    for (auto preset : presets) {
      supported_presets_.push_back(preset);
    }
  }

  void set_display(bool state);
  void set_turbo(bool state);

 protected:
  climate::ClimateTraits traits() override;
  void read_state_(const uint8_t *data, uint8_t size);
  void send_data_(const uint8_t *message, uint8_t size);
  void dump_message_(const char *title, const uint8_t *message, uint8_t size);
  uint8_t get_checksum_(const uint8_t *message, size_t size);

 private:
  // ... все константы те же ...

  uint8_t data_write_[47] = { /* ... */ };
  uint8_t data_read_[GREE_RX_BUFFER_SIZE] = {0};
  bool receiving_packet_ = false;

  std::vector<climate::ClimatePreset> supported_presets_{};
};

}  // namespace gree
}  // namespace esphome
