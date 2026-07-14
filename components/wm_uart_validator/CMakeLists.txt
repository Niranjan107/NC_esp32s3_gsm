idf_component_register(
    SRCS "wm_uart_validator.c"
    INCLUDE_DIRS "include"
    REQUIRES wm_uart soft_uart_rmt esp_timer esp_rom freertos driver log
)
