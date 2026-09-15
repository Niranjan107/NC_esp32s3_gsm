idf_component_register(
    SRCS "cmd_parser.c"
    INCLUDE_DIRS "include"
    REQUIRES json common config wm_uart ma_uart printer_uart battery ota mbedtls log gsm
)
