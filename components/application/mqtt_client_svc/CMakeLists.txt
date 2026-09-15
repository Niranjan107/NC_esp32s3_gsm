idf_component_register(
    SRCS "store_forward.c" "mqtt_client_svc.c"
    INCLUDE_DIRS "include"
    # No connectivity component here on purpose: this layer talks to the network
    # only through base/common's net_link interface, so it is portable to the GSM
    # product unchanged. Adding wifi_sta back would break that.
    # `config` is base, not connectivity - the store-and-forward on/off flag
    # lives with the other device settings, so this stays portable to GSM.
    REQUIRES mqtt common esp_event joltwallet__littlefs esp-tls wm_capture config
)
