idf_component_register(
    SRCS "ma_uart.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer nvs_flash wm_uart
)
