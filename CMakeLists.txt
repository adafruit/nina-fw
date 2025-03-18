# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# SDKCONFIG & SDKCONFIG_DEFAULTS for each target
set(SDKCONFIG ${CMAKE_SOURCE_DIR}/sdkconfig.${IDF_TARGET})

set(SDKCONFIG_DEFAULTS sdkconfig.defaults sdkconfig.defaults.${IDF_TARGET})
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  list(APPEND SDKCONFIG_DEFAULTS sdkconfig.debug)
endif ()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(CMAKE_BUILD_TYPE_DEBUG)
endif ()

# caused by esp-idf/components/bt/controller/esp32c6/bt.c:253:11: In function 'esp_bt_controller_log_init':
# error: 'task_create' may be used uninitialized [-Werror=maybe-uninitialized]
if (IDF_TARGET STREQUAL "esp32c6")
  add_compile_options(-Wno-maybe-uninitialized)
endif ()

project(nina-fw)
