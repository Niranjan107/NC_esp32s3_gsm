# The application layer is optional: it arrives in Step 4 of the GSM port, and
# every use of it in cmd_parser.c is already inside #ifdef CONFIG_NCLE_MQTT_ENABLE.
# Requiring the components unconditionally would make this file unbuildable until
# application/ exists, which would force Steps 3 and 4 to be done as one
# untestable change.
set(NCLE_APP_DEPS "")
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../application/mqtt_client_svc")
    list(APPEND NCLE_APP_DEPS mqtt_client_svc)
endif()
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../application/fota")
    list(APPEND NCLE_APP_DEPS fota)
endif()

idf_component_register(
    SRCS "cmd_parser.c"
    INCLUDE_DIRS "include"
    # No connectivity component here: this parser reaches the network only through
    # base/common's net_link, so the same file builds on the WiFi and GSM products.
    # (wifi_sta was removed when diag became connectivity-neutral.)
    REQUIRES json common config wm_uart ma_uart printer_uart battery ota mbedtls log
             ${NCLE_APP_DEPS}
)
