idf_component_register(
    SRCS "fota.c"
    INCLUDE_DIRS "."
    # mqtt_client_svc: FOTA suspends the broker session during the download to free
    # the RAM an HTTPS TLS record needs. Same layer, so this is not a layer breach.
    REQUIRES esp_https_ota esp_http_client esp-tls app_update esp_partition mqtt_client_svc
)
