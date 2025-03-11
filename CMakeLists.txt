# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# set SDKCONFIG for each IDF Target
set(SDKCONFIG ${CMAKE_SOURCE_DIR}/sdkconfig.${IDF_TARGET})

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(SDKCONFIG_DEFAULTS sdkconfig.defaults sdkconfig.debug)
endif ()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(CMAKE_BUILD_TYPE_DEBUG)
endif ()

project(nina-fw)
