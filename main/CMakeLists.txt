idf_component_register(SRCS CommandHandler.cpp http_client.c sketch.ino.cpp
                      REQUIRES bt esp_http_client spiffs SPIS
                      PRIV_INCLUDE_DIRS "."
                      INCLUDE_DIRS "." $ENV{IDF_PATH}/components/esp_netif
)
