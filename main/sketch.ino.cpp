/*
  This file is part of the Arduino NINA firmware.
  Copyright (c) 2018 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <rom/uart.h>

extern "C" {
  #include "esp_private/periph_ctrl.h"
  #include "soc/gpio_periph.h"
  #include "soc/periph_defs.h"
  // #include <driver/periph_ctrl.h>

  #include <driver/uart.h>
  #include <esp_bt.h>

  #include "esp_spiffs.h"
  #include "esp_log.h"
  #include <stdio.h>
  #include <sys/types.h>
  #include <dirent.h>
  #include "esp_partition.h"
}

#include <Arduino.h>

#include <SPIS.h>
#include <WiFi.h>

#include "CommandHandler.h"

#define SPI_BUFFER_LEN SPI_MAX_DMA_LEN

// UART debug is enabled on boot
int debug = 1;

//--------------------------------------------------------------------
// ADAFRUIT CHANGE
//--------------------------------------------------------------------
#define AIRLIFT 1 // Adafruit Airlift
#define NINA_PRINTF(...) do { if (debug) { ets_printf(__VA_ARGS__); } } while (0)

#if defined(CONFIG_IDF_TARGET_ESP32)
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

extern const struct __sFILE_fake __sf_fake_stdin;
extern const struct __sFILE_fake __sf_fake_stdout;
extern const struct __sFILE_fake __sf_fake_stderr;

// dev, dma, mosi, miso, sclk, cs, ready
SPISClass SPIS(VSPI_HOST, 1, AIRLIFT_MOSI, AIRLIFT_MISO, AIRLIFT_SCK, AIRLIFT_CS, AIRLIFT_BUSY);
#endif

#if defined(CONFIG_IDF_TARGET_ESP32C6)

// UART for BLE HCI
// CONFIG_BT_LE_HCI_UART_RTS_PIN and CONFIG_BT_LE_HCI_UART_CTS_PIN are defined in sdkconfig.defaults.BOARD
// and used by hci_driver_uart_config() in hci_driver_uart.c. It should matches with BUSY and BOOT pins.
#ifndef CONFIG_BT_LE_HCI_INTERFACE_USE_UART
#error "Please Enable Uart for HCI"
#endif

#if CONFIG_BT_LE_HCI_UART_CTS_PIN != 9
#error "CTS pin must be the same as BOOT pin"
#endif

// SPIS for WiFi
#define AIRLIFT_BUSY  CONFIG_BT_LE_HCI_UART_RTS_PIN // ready

#if defined(BOARD_FRUITJAM_C6)
  #define AIRLIFT_MOSI  21
  #define AIRLIFT_MISO  6
  #define AIRLIFT_SCK   22
  #define AIRLIFT_CS    7
#else
  #error "Board is not supported, please add -DBOARD=<board_name> to the build command"
#endif

// dev, dma, mosi, miso, sclk, cs, ready
SPISClass SPIS(SPI2_HOST, SPI_DMA_CH_AUTO,
               AIRLIFT_MOSI, AIRLIFT_MISO, AIRLIFT_SCK, AIRLIFT_CS, AIRLIFT_BUSY);
#endif

// prevent initArduino() to release BT memory
extern "C" bool btInUse() {
  return true;
}

//--------------------------------------------------------------------
//
//--------------------------------------------------------------------
uint8_t* commandBuffer;
uint8_t* responseBuffer;

void dumpBuffer(const char* label, uint8_t data[], int length) {
  ets_printf("%s: ", label);

  for (int i = 0; i < length; i++) {
    ets_printf("%02x", data[i]);
  }

  ets_printf("\r\n");
}

void setDebug(int d) {
  debug = d;

  if (debug) {
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[1], 0);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[3], 0);

    const char* default_uart_dev = "/dev/uart/0";
    _GLOBAL_REENT->_stdin  = fopen(default_uart_dev, "r");
    _GLOBAL_REENT->_stdout = fopen(default_uart_dev, "w");
    _GLOBAL_REENT->_stderr = fopen(default_uart_dev, "w");

    uart_div_modify(CONFIG_CONSOLE_UART_NUM, (APB_CLK_FREQ << 4) / 115200);

    // uartAttach();
    ets_install_uart_printf();
    uart_tx_switch(CONFIG_CONSOLE_UART_NUM);
  } else {
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[1], PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[3], PIN_FUNC_GPIO);

#if CONFIG_IDF_TARGET_ESP32
    _GLOBAL_REENT->_stdin  = (FILE*) &__sf_fake_stdin;
    _GLOBAL_REENT->_stdout = (FILE*) &__sf_fake_stdout;
    _GLOBAL_REENT->_stderr = (FILE*) &__sf_fake_stderr;
#endif

    ets_install_putc1(NULL);
    ets_install_putc2(NULL);
  }
}

void setupWiFi();
void setupBluetooth();

void setup() {
#ifndef CMAKE_BUILD_TYPE_DEBUG
  setDebug(0);
#endif

#if !AIRLIFT
  // put SWD and SWCLK pins connected to SAMD as inputs
  pinMode(15, INPUT);
  pinMode(21, INPUT);
#endif

  pinMode(AIRLIFT_CS, INPUT);
  if (digitalRead(AIRLIFT_CS) == LOW) {
    setupBluetooth();
  } else {
    setupWiFi();
  }
}

// #define UNO_WIFI_REV2

void setupBluetooth() {
  NINA_PRINTF("*** BLUETOOTH\n");

  periph_module_enable(PERIPH_UART1_MODULE);
  periph_module_enable(PERIPH_UHCI0_MODULE);

  esp_bt_controller_config_t btControllerConfig = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

#if defined(CONFIG_IDF_TARGET_ESP32)
#if defined(AIRLIFT)
  // TX GPIO1 & RX GPIO3 on ESP32 'hardware' UART
  // RTS on ESP_BUSY (GPIO33)
  // CTS on GPIO0 (GPIO0)
  uart_set_pin(UART_NUM_1, 1, 3, AIRLIFT_RTS, AIRLIFT_CTS);
#elif defined(UNO_WIFI_REV2)
  uart_set_pin(UART_NUM_1, 1, 3, 33, 0); // TX, RX, RTS, CTS
#elif defined(NANO_RP2040_CONNECT)
  uart_set_pin(UART_NUM_1, 1, 3, 33, 12); // TX, RX, RTS, CTS
#else
  uart_set_pin(UART_NUM_1, 23, 12, 18, 5);
#endif

  uart_set_hw_flow_ctrl(UART_NUM_1, UART_HW_FLOWCTRL_CTS_RTS, 5);

  btControllerConfig.hci_uart_no = UART_NUM_1;
#if defined(AIRLIFT)
  btControllerConfig.hci_uart_baudrate = 115200;
#elif defined(UNO_WIFI_REV2) || defined(NANO_RP2040_CONNECT)
  btControllerConfig.hci_uart_baudrate = 115200;
#else
  btControllerConfig.hci_uart_baudrate = 912600;
#endif

#elif defined(CONFIG_IDF_TARGET_ESP32C6)
  // UART is configured by CONFIG_BT_LE_HCI_UART_XYZ in sdkconfig.defaults.esp32c6
#endif

  esp_err_t ret = esp_bt_controller_init(&btControllerConfig);
  if (ESP_OK != ret) {
    setDebug(1);
    NINA_PRINTF("esp_bt_controller_init failed: 0x%x\n", ret);
    while (1) {}
  }

  while (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE);
  esp_bt_controller_enable(ESP_BT_MODE_BLE);

#if defined(CONFIG_IDF_TARGET_ESP32)
  esp_bt_sleep_enable();
#endif

  vTaskSuspend(NULL);

  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}

void setupWiFi() {
  NINA_PRINTF("WIFI ON\n");
  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
  SPIS.begin();

  esp_vfs_spiffs_conf_t conf = {
    .base_path = "/fs",
    .partition_label = "storage",
    .max_files = 20,
    .format_if_mount_failed = true
  };

  esp_err_t ret = esp_vfs_spiffs_register(&conf);
  (void) ret;

  if (WiFi.status() == WL_NO_SHIELD) {
    if (!debug) {
      setDebug(1);
    }
    NINA_PRINTF("*** NOSHIELD\n");
    while (1); // no shield
  }

  commandBuffer = (uint8_t*)heap_caps_malloc(SPI_BUFFER_LEN, MALLOC_CAP_DMA);
  responseBuffer = (uint8_t*)heap_caps_malloc(SPI_BUFFER_LEN, MALLOC_CAP_DMA);

  NINA_PRINTF("*** CommandHandler Begin\n");
  CommandHandler.begin();
}

void loop() {
  // wait for a command
  memset(commandBuffer, 0x00, SPI_BUFFER_LEN);
  int commandLength = SPIS.transfer(NULL, commandBuffer, SPI_BUFFER_LEN);

  if (commandLength == 0) {
    return;
  }

  if (debug) {
    dumpBuffer("COMMAND", commandBuffer, commandLength);
  }

  // process
  memset(responseBuffer, 0x00, SPI_BUFFER_LEN);
  int responseLength = CommandHandler.handle(commandBuffer, responseBuffer);

  SPIS.transfer(responseBuffer, NULL, responseLength);

  if (debug) {
    dumpBuffer("RESPONSE", responseBuffer, responseLength);
  }
}
