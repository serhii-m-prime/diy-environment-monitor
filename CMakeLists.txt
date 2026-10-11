cmake_minimum_required(VERSION 3.16)
# Only SPI/Wire from Arduino are used. No remote Arduino service components
# (Matter, speech recognition, RainMaker, Zigbee, etc.) are needed.
set(ENV{IDF_COMPONENT_MANAGER} "0")
set(MONITOR_ARDUINO_DIR "${CMAKE_CURRENT_LIST_DIR}/.pio/idf-core/packages/framework-arduinoespressif32"
    CACHE PATH "Arduino 3.3.12 source component")
list(APPEND EXTRA_COMPONENT_DIRS "${MONITOR_ARDUINO_DIR}")
# Build the application component and its declared dependencies, rather than
# every optional SDK component (test frameworks, consoles, transports, etc.).
set(COMPONENTS src)
if(EXISTS "$ENV{IDF_PATH}/components/__pio_env/CMakeLists.txt")
    # PlatformIO uses this generated component to obtain C/C++/ASM flags.
    list(APPEND COMPONENTS __pio_env)
endif()
set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/config/sdkconfig.common")
if(SDKCONFIG MATCHES "debug")
    list(APPEND SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/config/sdkconfig.debug")
else()
    list(APPEND SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/config/sdkconfig.release")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(environment_monitor)
