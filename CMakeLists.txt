# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

if (BOARD STREQUAL "fruitjam_c6")
  set(IDF_TARGET esp32c6 CACHE STRING "Target ESP-IDF board")
elseif (BOARD STREQUAL "esp32")
  set(IDF_TARGET esp32 CACHE STRING "Target ESP-IDF board")
else ()
  message(FATAL_ERROR "Unsupported BOARD: ${BOARD}. Supported boards are: fruitjam_c6, esp32")
endif ()

# SDKCONFIG & SDKCONFIG_DEFAULTS for each target
set(SDKCONFIG ${CMAKE_SOURCE_DIR}/sdkconfig.${IDF_TARGET})

# caused by esp-idf/components/bt/controller/esp32c6/bt.c:253:11: In function 'esp_bt_controller_log_init':
# error: 'task_create' may be used uninitialized [-Werror=maybe-uninitialized]
if (IDF_TARGET STREQUAL "esp32c6")
  add_compile_options(-Wno-maybe-uninitialized)
endif ()

set(SDKCONFIG_DEFAULTS sdkconfig.defaults sdkconfig.defaults.${BOARD})
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  list(APPEND SDKCONFIG_DEFAULTS sdkconfig.debug)
endif ()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(CMAKE_BUILD_TYPE_DEBUG)
endif ()

STRING(TOUPPER ${BOARD} BOARD_UPPER)
add_compile_definitions(
  BOARD_${BOARD_UPPER}
  )

project(nina-fw)
