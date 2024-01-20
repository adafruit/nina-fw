# - /!\ ------------------------------------------------------------------------
# Due to a limitation of arduino-esp32's build system, we cannot disable various
# libraries that are unneeded for the nina-fw firmware. As a workaround, this
# is a copy of the arduino-esp32 CMakeLists.txt with libraries removed.
#
# Bug: https://github.com/espressif/arduino-esp32/issues/6788
# ------------------------------------------------------------------------------

# Check ESP-IDF version and error out if it is not in the supported range.
#
# Note for arduino-esp32 developers: to bypass the version check locally,
# set ARDUINO_SKIP_IDF_VERSION_CHECK environment variable to 1. For example:
#   export ARDUINO_SKIP_IDF_VERSION_CHECK=1
#   idf.py build

set(CONFIG_ARDUINO_VARIANT "esp32")
set(CONFIG_AUTOSTART_ARDUINO 1)

set(min_supported_idf_version "4.4.0")
set(max_supported_idf_version "4.4.99")
set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}.${IDF_VERSION_PATCH}")

if ("${idf_version}" AND NOT "$ENV{ARDUINO_SKIP_IDF_VERSION_CHECK}")
  if (idf_version VERSION_LESS min_supported_idf_version)
    message(FATAL_ERROR "Arduino-esp32 can be used with ESP-IDF versions "
                        "between ${min_supported_idf_version} and ${max_supported_idf_version}, "
                        "but an older version is detected: ${idf_version}.")
  endif()
  if (idf_version VERSION_GREATER max_supported_idf_version)
    message(FATAL_ERROR "Arduino-esp32 can be used with ESP-IDF versions "
                        "between ${min_supported_idf_version} and ${max_supported_idf_version}, "
                        "but a newer version is detected: ${idf_version}.")
  endif()
endif()

set(CORE_SRCS
  arduino/cores/esp32/base64.cpp
  arduino/cores/esp32/cbuf.cpp
  arduino/cores/esp32/esp32-hal-adc.c
  arduino/cores/esp32/esp32-hal-bt.c
  arduino/cores/esp32/esp32-hal-cpu.c
  arduino/cores/esp32/esp32-hal-dac.c
  arduino/cores/esp32/esp32-hal-gpio.c
  arduino/cores/esp32/esp32-hal-i2c.c
  arduino/cores/esp32/esp32-hal-i2c-slave.c
  arduino/cores/esp32/esp32-hal-ledc.c
  arduino/cores/esp32/esp32-hal-matrix.c
  arduino/cores/esp32/esp32-hal-misc.c
  arduino/cores/esp32/esp32-hal-psram.c
  arduino/cores/esp32/esp32-hal-rgb-led.c
  arduino/cores/esp32/esp32-hal-sigmadelta.c
  arduino/cores/esp32/esp32-hal-spi.c
  arduino/cores/esp32/esp32-hal-time.c
  arduino/cores/esp32/esp32-hal-timer.c
  arduino/cores/esp32/esp32-hal-tinyusb.c
  arduino/cores/esp32/esp32-hal-touch.c
  arduino/cores/esp32/esp32-hal-uart.c
  arduino/cores/esp32/esp32-hal-rmt.c
  arduino/cores/esp32/Esp.cpp
  arduino/cores/esp32/FunctionalInterrupt.cpp
  arduino/cores/esp32/HardwareSerial.cpp
  arduino/cores/esp32/IPAddress.cpp
  arduino/cores/esp32/IPv6Address.cpp
  arduino/cores/esp32/libb64/cdecode.c
  arduino/cores/esp32/libb64/cencode.c
  arduino/cores/esp32/main.cpp
  arduino/cores/esp32/MD5Builder.cpp
  arduino/cores/esp32/Print.cpp
  arduino/cores/esp32/stdlib_noniso.c
  arduino/cores/esp32/Stream.cpp
  arduino/cores/esp32/StreamString.cpp
  arduino/cores/esp32/Tone.cpp
  arduino/cores/esp32/HWCDC.cpp
  arduino/cores/esp32/USB.cpp
  arduino/cores/esp32/USBCDC.cpp
  arduino/cores/esp32/USBMSC.cpp
  arduino/cores/esp32/FirmwareMSC.cpp
  arduino/cores/esp32/firmware_msc_fat.c
  arduino/cores/esp32/wiring_pulse.c
  arduino/cores/esp32/wiring_shift.c
  arduino/cores/esp32/WMath.cpp
  arduino/cores/esp32/WString.cpp
  )

set(LIBRARY_SRCS "")

set(includedirs
  arduino/variants/${CONFIG_ARDUINO_VARIANT}/
  arduino/cores/esp32/
  )

set(srcs ${CORE_SRCS})
set(priv_includes arduino/cores/esp32/libb64)
set(requires spi_flash mbedtls mdns esp_adc_cal wifi_provisioning nghttp wpa_supplicant)
set(priv_requires fatfs nvs_flash app_update spiffs bootloader_support openssl bt esp_ipc esp_hid)

idf_component_register(INCLUDE_DIRS ${includedirs} PRIV_INCLUDE_DIRS ${priv_includes} SRCS ${srcs} REQUIRES ${requires} PRIV_REQUIRES ${priv_requires})

if(NOT CONFIG_FREERTOS_HZ EQUAL 1000 AND NOT "$ENV{ARDUINO_SKIP_TICK_CHECK}")
    # See delay() in cores/esp32/esp32-hal-misc.c.
    message(FATAL_ERROR "esp32-arduino requires CONFIG_FREERTOS_HZ=1000 "
                        "(currently ${CONFIG_FREERTOS_HZ})")
endif()

string(TOUPPER ${CONFIG_ARDUINO_VARIANT} idf_target_caps)
string(REPLACE "-" "_" idf_target_for_macro "${idf_target_caps}")
target_compile_options(${COMPONENT_TARGET} PUBLIC
    -DARDUINO=10812
    -DARDUINO_${idf_target_for_macro}_DEV
    -DARDUINO_ARCH_ESP32
    -DARDUINO_BOARD="${idf_target_caps}_DEV"
    -DARDUINO_VARIANT="${CONFIG_ARDUINO_VARIANT}"
    -DESP32)

if(CONFIG_AUTOSTART_ARDUINO)
    # in autostart mode, arduino-esp32 contains app_main() function and needs to
    # reference setup() and loop() in the main component. If we add main
    # component to priv_requires then we create a large circular dependency
    # (arduino-esp32 -> main -> arduino-esp32) and can get linker errors, so
    # instead we add setup() and loop() to the undefined symbols list so the
    # linker will always include them.
    #
    # (As they are C++ symbol, we need to add the C++ mangled names.)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-u _Z5setupv -u _Z4loopv")
endif()

# This function adds a dependency on the given component if the component is included into the build.
function(maybe_add_component component_name)
    idf_build_get_property(components BUILD_COMPONENTS)
    if (${component_name} IN_LIST components)
        idf_component_get_property(lib_name ${component_name} COMPONENT_LIB)
        target_link_libraries(${COMPONENT_LIB} PUBLIC ${lib_name})
    endif()
endfunction()

maybe_add_component(esp-dsp)

if(CONFIG_ESP_RMAKER_WORK_QUEUE_TASK_STACK)
    maybe_add_component(esp_rainmaker)
    maybe_add_component(qrcode)
endif()
if(IDF_TARGET MATCHES "esp32s2|esp32s3" AND CONFIG_TINYUSB_ENABLED)
    maybe_add_component(arduino_tinyusb)
endif()
if(NOT CONFIG_ARDUINO_SELECTIVE_COMPILATION OR CONFIG_ARDUINO_SELECTIVE_ArduinoOTA)
    maybe_add_component(esp_https_ota)
endif()
if(NOT CONFIG_ARDUINO_SELECTIVE_COMPILATION OR CONFIG_ARDUINO_SELECTIVE_LITTLEFS)
    maybe_add_component(esp_littlefs)
endif()
