idf_component_register(
    SRCS "gsm.c" "gsm_task.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_timer
)
