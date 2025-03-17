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

// Equivalent to #include "esp_log.h" when using Arduino.
// #include "esp32-hal-log.h"

#include "esp_private/periph_ctrl.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_bt.h"
#include "soc/gpio_periph.h"
#include "soc/periph_defs.h"

#include "esp_spiffs.h"
#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include "esp_partition.h"

#include <Arduino.h>

#include <SPIS.h>
#include <WiFi.h>

#include "CommandHandler.h"

#define SPI_BUFFER_LEN SPI_MAX_DMA_LEN

// UART debug is enabled on boot
int debug = 1;

#if defined(CONFIG_IDF_TARGET_ESP32)
extern const struct __sFILE_fake __sf_fake_stdin;
extern const struct __sFILE_fake __sf_fake_stdout;
extern const struct __sFILE_fake __sf_fake_stderr;
#endif

#if defined(CONFIG_IDF_TARGET_ESP32C6)
#ifndef CONFIG_BT_LE_HCI_INTERFACE_USE_UART
#error "Please Enable Uart for HCI"
#endif
#endif

// ADAFRUIT-CHANGE: AirLift conditionalization
#define AIRLIFT 1

#define NINA_PRINTF(...) do { if (debug) { ets_printf(__VA_ARGS__); } } while (0)

// Adafruit: prevent initArduino() to release BT memory
extern "C" bool btInUse() {
  return true;
}

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

  // put SWD and SWCLK pins connected to SAMD as inputs
  pinMode(15, INPUT);
  pinMode(21, INPUT);

  pinMode(5, INPUT);
  if (digitalRead(5) == LOW) {
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
  // uart_set_pin(UART_NUM_1, 1, 3, 33, 0);
  uart_set_pin(UART_NUM_1, 14, 18, 33, 0);
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
