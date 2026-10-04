idf_component_register(SRCS "robo_sumo.c" "servo.c" "bluetooth.c" INCLUDE_DIRS "."
PRIV_REQUIRES bt nvs_flash
REQUIRES esp_driver_mcpwm)
