idf_component_register(
    SRCS "ota.c"
    INCLUDE_DIRS "."
    REQUIRES esp_app_format app_update esp_partition log
)
