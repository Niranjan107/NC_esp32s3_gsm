idf_component_register(
    # wm_capture.c moved to components/application/wm_capture - it served the
    # cloud message only, which is not base's business. esp_timer stays: the
    # cycle-timing code in common.c uses it.
    SRCS "common.c" "net_link.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer log
)
