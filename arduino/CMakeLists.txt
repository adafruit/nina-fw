idf_component_register(SRCS
  cores/esp32/delay.c
  cores/esp32/IPAddress.cpp
  cores/esp32/itoa.c
  cores/esp32/main.cpp
  cores/esp32/Print.cpp
  cores/esp32/Stream.cpp
  cores/esp32/WInterrupts.c
  cores/esp32/wiring_analog.c
  cores/esp32/wiring.c
  cores/esp32/wiring_digital.c
  cores/esp32/WMath.cpp
  cores/esp32/WString.cpp

  libraries/ArduinoBearSSL/src/aiotc_profile.c
  libraries/ArduinoBearSSL/src/ArduinoBearSSL.cpp
  libraries/ArduinoBearSSL/src/BearSSLClient.cpp
  libraries/ArduinoBearSSL/src/SHA1.cpp
  libraries/ArduinoBearSSL/src/SHA256.cpp
  libraries/ArduinoBearSSL/src/SHA.cpp

  libraries/ArduinoECCX08/src/ECCX08.cpp
  libraries/ArduinoECCX08/src/utility/ASN1Utils.cpp
  libraries/ArduinoECCX08/src/utility/ECCX08CSR.cpp
  libraries/ArduinoECCX08/src/utility/ECCX08JWS.cpp
  libraries/ArduinoECCX08/src/utility/ECCX08SelfSignedCert.cpp
  libraries/ArduinoECCX08/src/utility/PEMUtils.cpp
  libraries/ArduinoECCX08/src/utility/sha1.c

  libraries/SPIS/src/SPIS.cpp

  libraries/WiFi/src/WiFiClient.cpp
  libraries/WiFi/src/WiFi.cpp
  libraries/WiFi/src/WiFiServer.cpp
  libraries/WiFi/src/WiFiSSLClient.cpp
  libraries/WiFi/src/WiFiUdp.cpp
  libraries/Wire/src/Wire.cpp

  INCLUDE_DIRS
  cores/esp32
  libraries/ArduinoBearSSL/src
  libraries/ArduinoECCX08/src
  libraries/SPIS/src
  libraries/WiFi/src
  libraries/Wire/src

  PRIV_INCLUDE_DIRS
  cores/esp32
  libraries/ArduinoBearSSL/src
  libraries/ArduinoECCX08/src
  libraries/SPIS/src
  libraries/WiFi/src
  libraries/Wire/src
)
