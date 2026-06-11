import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import climate, uart
from esphome.const import CONF_ID, CONF_SUPPORTED_PRESETS

# Корректный импорт перечисления ClimatePreset
from esphome.components.climate import ClimatePreset

DEPENDENCIES = ["climate", "uart"]

gree_ns = cg.esphome_ns.namespace("gree")
GreeClimate = gree_ns.class_(
    "GreeClimate", climate.Climate, cg.PollingComponent, uart.UARTDevice
)

# Используем актуальный метод climate_schema и указываем в нём наш класс
CONFIG_SCHEMA = climate.climate_schema(GreeClimate).extend(
    {
        cv.GenerateID(): cv.declare_id(GreeClimate),
        cv.Optional(CONF_SUPPORTED_PRESETS): cv.ensure_list(
            cv.enum(
                {
                    # 👇 Ключевое исправление: используем атрибуты перечисления ClimatePreset
                    "NONE": ClimatePreset.CLIMATE_PRESET_NONE,
                    "BOOST": ClimatePreset.CLIMATE_PRESET_BOOST,
                    "SLEEP": ClimatePreset.CLIMATE_PRESET_SLEEP,
                    "ECO": ClimatePreset.CLIMATE_PRESET_ECO,
                    "AWAY": ClimatePreset.CLIMATE_PRESET_AWAY,
                },
                upper=True,
            )
        ),
    }
).extend(cv.polling_component_schema("10s")).extend(uart.UART_DEVICE_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    await uart.register_uart_device(var, config)
    if CONF_SUPPORTED_PRESETS in config:
        # Передаём список пресетов в C++ компонент
        cg.add(var.set_supported_presets(config[CONF_SUPPORTED_PRESETS]))
