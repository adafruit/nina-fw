#ifndef BOARD_H
#define BOARD_H

// SPIS for WiFi
#define AIRLIFT_MOSI  14
#define AIRLIFT_MISO  23
#define AIRLIFT_SCK   18
#define AIRLIFT_CS    5
#define AIRLIFT_BUSY  33 // ready

// UART for BLE HCI
#define AIRLIFT_RTS   AIRLIFT_BUSY
#define AIRLIFT_CTS   0 // BOOT PIN

// #define CONFIG_BT_LE_HCI_UART_RTS_PIN 33 // ESP_BUSY (ready)
// #define CONFIG_BT_LE_HCI_UART_CTS_PIN 0  // GPIO0

#endif
