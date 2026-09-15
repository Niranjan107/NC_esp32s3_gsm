idf_component_register(
    SRCS "gsm.c" "gsm_task.c"
    INCLUDE_DIRS "include"
    # esp_netif + esp_event: the modem becomes a real network interface (PPP),
    # not an AT-command HTTP wrapper. nvs_flash: runtime APN override.
    # lwip + esp_http_client: the Step 1 self-test uses the STANDARD stack
    # (esp_ping, stock esp_http_client) to prove this is a real socket
    # interface rather than an AT-command wrapper.
    REQUIRES driver freertos esp_timer esp_netif esp_event nvs_flash
             lwip esp_http_client
)
