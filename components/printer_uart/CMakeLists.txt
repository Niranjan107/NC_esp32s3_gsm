idf_component_register(
    SRCS "printer_uart.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer nvs_flash
)
