# CMakeLists.txt for NCLite ESP32-S3 Main Application
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        nvs_flash
        esp_timer
    PRIV_REQUIRES
        wm_uart
        ma_uart
        cmd_parser
        printer_uart
        ble_spp
)
