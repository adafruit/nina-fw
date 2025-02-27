idf_component_register(SRCS
  # Arduino core sources needed.
  arduino-esp32/cores/esp32/esp32-hal-gpio.c
  arduino-esp32/cores/esp32/esp32-hal-ledc.c
  arduino-esp32/cores/esp32/esp32-hal-misc.c
  arduino-esp32/cores/esp32/esp32-hal-time.c
  arduino-esp32/cores/esp32/FunctionalInterrupt.cpp
  arduino-esp32/cores/esp32/IPAddress.cpp
  arduino-esp32/cores/esp32/main.cpp
  arduino-esp32/cores/esp32/Print.cpp
  arduino-esp32/cores/esp32/stdlib_noniso.c
  arduino-esp32/cores/esp32/Stream.cpp
  arduino-esp32/cores/esp32/Stream.cpp
  arduino-esp32/cores/esp32/WMath.cpp
  arduino-esp32/cores/esp32/WString.cpp

  # Use arduino-esp32 libraries instead of our own versions when possible

  arduino-esp32/libraries/Network/src/NetworkClient.cpp
  arduino-esp32/libraries/Network/src/NetworkClient.h
  arduino-esp32/libraries/Network/src/NetworkEvents.cpp
  arduino-esp32/libraries/Network/src/NetworkEvents.h
  arduino-esp32/libraries/Network/src/Network.h
  arduino-esp32/libraries/Network/src/NetworkInterface.cpp
  arduino-esp32/libraries/Network/src/NetworkInterface.h
  arduino-esp32/libraries/Network/src/NetworkManager.cpp
  arduino-esp32/libraries/Network/src/NetworkManager.h
  arduino-esp32/libraries/Network/src/NetworkServer.cpp
  arduino-esp32/libraries/Network/src/NetworkServer.h
  arduino-esp32/libraries/Network/src/NetworkUdp.cpp

  arduino-esp32/libraries/NetworkClientSecure/src/NetworkClientSecure.cpp
  arduino-esp32/libraries/NetworkClientSecure/src/NetworkClientSecure.h
  arduino-esp32/libraries/NetworkClientSecure/src/ssl_client.cpp
  arduino-esp32/libraries/NetworkClientSecure/src/ssl_client.h

  arduino-esp32/libraries/WiFi/src/AP.cpp
  arduino-esp32/libraries/WiFi/src/STA.cpp
  arduino-esp32/libraries/WiFi/src/WiFiAP.cpp
  arduino-esp32/libraries/WiFi/src/WiFiAP.h
  arduino-esp32/libraries/WiFi/src/WiFiClient.h
  arduino-esp32/libraries/WiFi/src/WiFi.cpp
  arduino-esp32/libraries/WiFi/src/WiFiGeneric.cpp
  arduino-esp32/libraries/WiFi/src/WiFiGeneric.h
  arduino-esp32/libraries/WiFi/src/WiFi.h
  arduino-esp32/libraries/WiFi/src/WiFiMulti.cpp
  arduino-esp32/libraries/WiFi/src/WiFiMulti.h
  arduino-esp32/libraries/WiFi/src/WiFiScan.cpp
  arduino-esp32/libraries/WiFi/src/WiFiScan.h
  arduino-esp32/libraries/WiFi/src/WiFiServer.h
  arduino-esp32/libraries/WiFi/src/WiFiSTA.cpp
  arduino-esp32/libraries/WiFi/src/WiFiSTA.h
  arduino-esp32/libraries/WiFi/src/WiFiType.h
  arduino-esp32/libraries/WiFi/src/WiFiUdp.h

  arduino-esp32/libraries/Wire/src/Wire.cpp

  INCLUDE_DIRS
  arduino-esp32/cores/esp32
  # For pins_arduino.h
  arduino-esp32/variants/esp32
  arduino-esp32/libraries/Network/src
  arduino-esp32/libraries/NetworkClientSecure/src
  arduino-esp32/libraries/WiFi/src
  arduino-esp32/libraries/Wire/src
  libraries/SPIS/src

  PRIV_INCLUDE_DIRS
  arduino-esp32/cores/esp32
  # For pins_arduino.h
  arduino-esp32/variants/esp32
  arduino-esp32/libraries/Network/src
  arduino-esp32/libraries/NetworkClientSecure/src
  arduino-esp32/libraries/WiFi/src
  arduino-esp32/libraries/Wire/src
  libraries/SPIS/src

  REQUIRES
  bt
  esp_driver_ledc
  esp_driver_gpio
  esp_driver_gptimer
  esp_driver_spi
  esp_driver_uart
  esp_eth
  esp_http_client
  esp_partition
  esp_netif
  esp_ringbuf
  esp_timer
  mbedtls
  nvs_flash
  soc
  spiffs
  wpa_supplicant
)

add_compile_definitions(
  ARDUINO
  ARDUINO_EVENT_RUNNING_CORE=1
  ARDUINO_RUNNING_CORE=1
  )

# If we needed to use LWIP_HOOK_IP4_INPUT (see CommandHandler.cpp), add a hook this way.
# idf_component_get_property(lwip lwip COMPONENT_LIB)
# target_compile_options(${lwip} PRIVATE "-I${PROJECT_DIR}/main")
# target_compile_definitions(${lwip} PRIVATE "-DESP_IDF_LWIP_HOOK_FILENAME=\"lwip_hooks.h\"")

add_subdirectory(libraries/SPIS)
