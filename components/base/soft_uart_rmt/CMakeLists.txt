idf_component_register(
    SRCS "soft_uart_rmt.c" "rmt_decoder.c"
    INCLUDE_DIRS "include" "."
    REQUIRES driver esp_driver_rmt esp_timer esp_rom freertos
)
