cmake_minimum_required(VERSION 3.22)

# This project always builds for the Air ESP32.
set(IDF_TARGET "esp32")
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../common")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(drone_air_esp32)
