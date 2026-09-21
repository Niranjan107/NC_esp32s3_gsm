idf_component_register(
    SRCS "wifi_sta.c" "wifi_sta_task.c" "wifi_sta_config.c"
    INCLUDE_DIRS "include"
    # `common` for net_link.h - this component announces itself as a usable link
    # so the application layer never has to know WiFi exists.
    REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_coex common
    # PRIVATE on purpose: wifi_sta.c registers its own commands with cmd_parser,
    # but wifi_sta.h exposes none of that. cmd_parser still REQUIRES wifi_sta for
    # `diag` (until Task 10 makes diag connectivity-neutral), so a PUBLIC require
    # here would be a dependency cycle.
    PRIV_REQUIRES cmd_parser json mqtt_client_svc
)
