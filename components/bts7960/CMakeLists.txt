idf_component_register(
    SRCS "bts7960.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_mcpwm esp_driver_gpio
)
