idf_component_register(
    SRCS "wm_uart.c"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash esp_timer soft_uart_rmt
)
