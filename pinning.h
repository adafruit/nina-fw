#ifdef OPTIMISED_PINNING
  #define VSPI_DMACHANNEL 1
  #define VSPI_MOSI       19 /* IO0 */
  #define VSPI_MISO       23 /* IO1 */
  #define VSPI_SCLK       18 /* CLK */
  #define VSPI_SS         5  /* CS */
  #define VSPI_READY      21 /* IO3 */
  #define VSPI_WP         22 /* IO2 */

  #define UART1_TX        1
  #define UART1_RX        3
  #define UART1_RTS       33
  #define UART1_CTS       32
  #define UART1_BAUD      921600

//BT_MODE_SENSE is not used with this pinning

#else
  #define VSPI_DMACHANNEL 1
  #define VSPI_MOSI       14
  #define VSPI_MISO       23
  #define VSPI_SCLK       18
  #define VSPI_SS         5
  #define VSPI_READY      33

  #define BT_MODE_SENSE   5
  #define SWD_SAMD        15
  #define SWCLK_SAMD      21

  #ifdef UNO_WIFI_REV2
    #define UART1_TX        1
    #define UART1_RX        3
    #define UART1_RTS       33
    #define UART1_CTS       5
    #define UART1_BAUD      115200
  #else
    #define UART1_TX        23
    #define UART1_RX        12
    #define UART1_RTS       18
    #define UART1_CTS       5
    #define UART1_BAUD      921600
  #endif
#endif
