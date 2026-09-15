# ESP-IDF Project CMakeLists.txt for NCLite ESP32-S3

cmake_minimum_required(VERSION 3.16)

# Set the project components
# Components are organised in three layers (see docs/GSM_PORT_PLAN.md):
#   base/         - meters, printer, BLE, commands, settings, chunked OTA.
#                   Identical in every CLV4 product; never edited per variant.
#   connectivity/ - how the device reaches the internet. THE ONLY LAYER THAT
#                   DIFFERS between products: gsm here, wifi_sta in the WiFi
#                   project.
#   application/  - what the device does with the internet (MQTT publish,
#                   store-and-forward, weight merge, HTTPS FOTA). Portable:
#                   copied from the WiFi product unchanged.
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/components/base"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/connectivity"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/application"
)

# This board has no display. esp_lcd is unused, and its RGB panel driver makes
# the xtensa GCC 14.2 build crash (internal compiler error in the `ira` pass on
# esp_lcd_panel_rgb.c), so exclude it from the build entirely.
# Same exclusion as the WiFi sister project - same board, same toolchain.
set(EXCLUDE_COMPONENTS esp_lcd)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(NCLite_ESP32S3)

# Firmware version
# Bumped from 2.0.0.1000 (pre-port) once GSM/PPP, the three-layer restructure
# and the shared cloud layer landed - a build this different must not keep the
# old number, or a bug report cannot be tied to a firmware.
set(NCLE_FIRMWARE_VERSION "2.1.0.1000")
# idf_build_set_property, not add_compile_definitions: the latter only reaches
# targets defined after this point, which silently misses components. Matches
# the WiFi product.
idf_build_set_property(COMPILE_DEFINITIONS "NCLE_FIRMWARE_VERSION=\"${NCLE_FIRMWARE_VERSION}\"" APPEND)
