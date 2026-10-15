
ESPHome Gree HVAC (Aeronik) Component
Полнофункциональный компонент ESPHome для управления кондиционерами Gree и их OEM-аналогами (включая Aeronik) через UART.

Возможности
🌡️ Полный контроль температуры и режимов (Cool, Heat, Dry, Fan, Auto, Off)
🌬️ Управление скоростью вентилятора
🔄 Управление положением жалюзи (Верх, Центр, Низ, Качение)
📺 Включение/выключение дисплея (подсветки)
🚀 Режим Турбо
🛡️ Защита от битых и слишком длинных пакетов UART
🔄 Двусторонняя синхронизация с физическим ИК-пультом
Подключение
Плата ESP	Кондиционер
TX	RX
RX	TX
GND	GND
⚠️ Общая земля (GND) обязательна. Без неё связь может какое-то время работать за счёт случайных обходных путей, но сигнал получается слабым и с помехами: пакеты теряются, а на ESP8266 возможны самопроизвольные перезагрузки.

⚠️ Пины ESP рассчитаны на 3.3 В. Если UART кондиционера работает на 5 В, используйте преобразователь уровней (обе стороны должны быть запитаны: LV — 3.3 В, HV — 5 В).

Выбор пинов
Желателен UART. Программный UART (например, D5/D6 на ESP8266) при 4800 бод с контролем чётности занимает процессор в прерываниях и может привести к перезагрузкам Hardware WDT.

Плата	TX	RX	Примечание
ESP32-C3 Super Mini (рекомендуется)	GPIO20	GPIO21	Логи идут через встроенный USB, UART свободен. Не используйте GPIO2, 8, 9 (strapping) и 18/19 (USB)
ESP8266 (требует проверки!) (Wemos D1 mini)	TX (GPIO1)	RX (GPIO3)	Обязательно logger: baud_rate: 0. Прошивка по USB только с отключённым кондиционером
На ESP8266 не используйте альтернативные пины UART0 D8/D7 (GPIO15/GPIO13): линия кондиционера подтягивает GPIO15 к питанию, и плата не загружается (boot mode:(7,7), waiting for host).

На ESP32-C3 пины назначаются через матрицу GPIO, поэтому, если связи нет, можно просто поменять местами tx_pin и rx_pin в конфиге без перепайки.

Установка
Добавьте этот код в ваш конфигурационный файл ESPHome.

Для ESP32-C3 Super Mini:

esp32:
  board: esp32-c3-devkitm-1
  framework:
    type: esp-idf

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  output_power: 8.5dB  # у Super Mini слабая антенна, без этого Wi-Fi нестабилен

uart:
  id: ac_uart
  tx_pin: GPIO20
  rx_pin: GPIO21
  baud_rate: 4800
  parity: EVEN
Для ESP8266 (Wemos D1 mini) (требует проверки):

esp8266:
  board: d1_mini

logger:
  baud_rate: 0  # освобождаем UART0 для кондиционера

uart:
  id: ac_uart
  tx_pin: TX  # GPIO1
  rx_pin: RX  # GPIO3
  baud_rate: 4800
  parity: EVEN
Общая часть для обеих плат:

external_components:
  - source:
      type: git
      url: https://github.com/Satlax/esphome_gree_hvac_aeronik
      ref: v1.1.0
    components: [gree]
    refresh: 0s

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
