# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# Ground defaults to ESP32-S3; `idf.py set-target esp32` selects Air's ESP32.
if(NOT DEFINED IDF_TARGET)
    set(IDF_TARGET "esp32s3")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_controller)
