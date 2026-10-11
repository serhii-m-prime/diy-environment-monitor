cmake_minimum_required(VERSION 3.16)
# Only SPI/Wire from Arduino are used. No remote Arduino service components
# (Matter, speech recognition, RainMaker, Zigbee, etc.) are needed.
set(ENV{IDF_COMPONENT_MANAGER} "0")
set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/config/sdkconfig.common")
if(SDKCONFIG MATCHES "debug")
    list(APPEND SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/config/sdkconfig.debug")
else()
    list(APPEND SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/config/sdkconfig.release")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(environment_monitor)
