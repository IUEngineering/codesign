/*!
 *  \file    serialF0.h
 *  \author  Wim Dolman (<a href="mailto:w.e.dolman@hva.nl">w.e.dolman@hva.nl</a>)
 *           Edwin Boer (for a few bugfixes)
 *  \date    20200321
 *  \version 1.8.1
 *
 *  \brief   Serial interface voor HvA-Xmegaboard
 *
 *  \details This serial interface doesn't use the drivers of Atmel
 *           The interface uses two \e non circulair buffers for sending and 
 *           receiving the data.
 *           It is based om md_serial.c from J.D.Bakker.
 *
 *           It is a serial interface for the HvA-Xmegaboard (Version 2) with a
 *           Xmega256a3u and Xmega32a4u for programming and the serial interface.
 *           You can use the standard printf, putchar, puts, scanf, getchar, ...
 *           functions.
 *
 *           The baud rate is 115200
 */
 
#ifndef SERIALF0_H_
#define SERIALF0_H_

  #include <stdio.h>

  #define  BAUD_115K2          115200UL    //!< Baud rate 115200
  #define  BAUD_57K6           57600UL     //!< Baud rate 57600
  #define  BAUD_38K4           38400UL     //!< Baud rate 38400

  #define  UART_DOUBLE_CLK      1          //!< Double clock speed true
  #define  UART_NO_DOUBLE_CLK   0          //!< Double clock speed false

  #define TXBUF_DEPTH_F0    100      //!<  size of transmit buffer
  #define RXBUF_DEPTH_F0    100      //!<  size of receive buffer

  #define UART_NO_DATA      0x0100                      //!< Macro UART_NO_DATA is returned by uart_getc when no data is present
  #define clear_screen()    printf("\e[H\e[2J\e[3J");   //!< Macro to reset and clear the terminal

  char     *getline(char* buf,  uint16_t len);
  void      init_stream(uint32_t f_cpu, uint32_t baud);
  uint16_t  uartF0_getc(void);
  void      uartF0_putc(uint8_t data);
  void      uartF0_puts(char *s);

#endif // SERIALF0_H_ 
