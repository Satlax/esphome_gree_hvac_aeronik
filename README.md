# ESPHome Gree HVAC (Aeronik) Component

Полнофункциональный компонент ESPHome для управления кондиционерами Gree и их OEM-аналогами (включая Aeronik) через UART.

## Возможности
- 🌡️ Полный контроль температуры и режимов (Cool, Heat, Dry, Fan, Auto, Off)
- 🌬️ Управление скоростью вентилятора
- 🔄 Управление положением жалюзи (Верх, Центр, Низ, Качение)
- 📺 Включение/выключение дисплея (подсветки)
- 🚀 Режим Турбо
- 🛡️ Защита от сбоев UART и Soft WDT
- 🔄 Двусторонняя синхронизация с физическим ИК-пультом

## Установка
Добавьте этот код в ваш конфигурационный файл ESPHome:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/Satlax/esphome_gree_hvac_aeronik
      ref: v1.1.0
    components: [gree]
    refresh: 0s

uart:
  id: ac_uart
  tx_pin: D5  # Укажите ваш TX пин
  rx_pin: D6  # Укажите ваш RX пин
  baud_rate: 4800
  parity: EVEN

climate:
  - platform: gree
    name: "Aeronik AC"
    id: aeronik_climate
    uart_id: ac_uart
    update_interval: 10s
    visual:
      min_temperature: 16
      max_temperature: 30
      temperature_step: 1.0

select:
  - platform: template
    name: "AC Swing Mode"
    id: ac_swing_select
    options:
      - "Верх"
      - "Центр"
      - "Низ"
      - "Качение"
    lambda: |-
      if (id(aeronik_climate).get_swing_mode() == 0x20) return std::string("Верх");
      if (id(aeronik_climate).get_swing_mode() == 0x40) return std::string("Центр");
      if (id(aeronik_climate).get_swing_mode() == 0x60) return std::string("Низ");
      if (id(aeronik_climate).get_swing_mode() == 0x10) return std::string("Качение");
      return std::string("Верх");
    set_action:
      then:
        - lambda: |-
            if (x == "Верх") id(aeronik_climate).set_swing_mode(0x20);
            else if (x == "Центр") id(aeronik_climate).set_swing_mode(0x40);
            else if (x == "Низ") id(aeronik_climate).set_swing_mode(0x60);
            else if (x == "Качение") id(aeronik_climate).set_swing_mode(0x10);

  - platform: template
    name: "AC Fan Mode"
    id: ac_fan_select
    options:
      - "Авто"
      - "Низкий"
      - "Средний"
      - "Высокий"
    lambda: |-
      if (id(aeronik_climate).fan_mode == climate::CLIMATE_FAN_AUTO) return std::string("Авто");
      if (id(aeronik_climate).fan_mode == climate::CLIMATE_FAN_LOW) return std::string("Низкий");
      if (id(aeronik_climate).fan_mode == climate::CLIMATE_FAN_MEDIUM) return std::string("Средний");
      if (id(aeronik_climate).fan_mode == climate::CLIMATE_FAN_HIGH) return std::string("Высокий");
      return std::string("Авто");
    set_action:
      then:
        - lambda: |-
            climate::ClimateCall call = id(aeronik_climate).make_call();
            if (x == "Авто") call.set_fan_mode(climate::CLIMATE_FAN_AUTO);
            else if (x == "Низкий") call.set_fan_mode(climate::CLIMATE_FAN_LOW);
            else if (x == "Средний") call.set_fan_mode(climate::CLIMATE_FAN_MEDIUM);
            else if (x == "Высокий") call.set_fan_mode(climate::CLIMATE_FAN_HIGH);
            call.perform();

switch:
  - platform: template
    name: "AC Display"
    id: ac_display_switch
    lambda: |-
      return id(aeronik_climate).get_display_state();
    turn_on_action:
      - lambda: id(aeronik_climate).set_display(true);
    turn_off_action:
      - lambda: id(aeronik_climate).set_display(false);

  - platform: template
    name: "AC Turbo"
    id: ac_turbo_switch
    lambda: |-
      return id(aeronik_climate).get_turbo_state();
    turn_on_action:
      - lambda: id(aeronik_climate).set_turbo(true);
    turn_off_action:
      - lambda: id(aeronik_climate).set_turbo(false);
