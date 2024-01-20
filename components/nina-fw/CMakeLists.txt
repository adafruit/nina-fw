idf_component_register(
  SRCS
  libraries/SPIS/src/SPIS.cpp
  libraries/WiFi/src/WiFi.cpp
  libraries/WiFi/src/WiFiClient.cpp
  libraries/WiFi/src/WiFiSSLClient.cpp
  libraries/WiFi/src/WiFiServer.cpp
  libraries/WiFi/src/WiFiUdp.cpp

  INCLUDE_DIRS
  libraries/SPIS/src
  libraries/WiFi/src

  REQUIRES
  arduino
)
