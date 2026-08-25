cmake_minimum_required(VERSION 3.22)

# This project always builds for the Ground ESP32-S3.
set(IDF_TARGET "esp32s3")
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../common")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(drone_ground_s3)
