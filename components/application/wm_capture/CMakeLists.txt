# Weight capture + merge into the cloud MA message.
#
# APPLICATION layer, not base: this exists only to put the weigher's value inside
# the MQTT copy of the reading. The base CLV4 product has no cloud path and no
# wm_capture at all, so keeping it in base/common would force every product to
# carry code it never uses.
#
# It depends on nothing from base. The base drivers reach it the other way round,
# through the generic ma_uart frame-start and wm_uart value callbacks that main.c
# wires up - so base never depends on this component.
idf_component_register(
    SRCS "wm_capture.c"
    INCLUDE_DIRS "."
    REQUIRES esp_timer
)
