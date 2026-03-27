# ESP-IDF Project CMakeLists.txt for NCLite ESP32-S3

cmake_minimum_required(VERSION 3.16)

# Set the project components
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(NCLite_ESP32S3)

# Firmware version
set(NCLE_FIRMWARE_VERSION "2.0.0.1000")
add_compile_definitions(NCLE_FIRMWARE_VERSION="${NCLE_FIRMWARE_VERSION}")
