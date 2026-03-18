#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_

#include <stdint.h>

/* Rozmiary buforów */
#ifndef UART_RX_BUF_SIZE
#define UART_RX_BUF_SIZE     1024
#endif

#ifndef UART_TX_BUF_SIZE
#define UART_TX_BUF_SIZE     2048
#endif

#ifndef ACCEL_BUFFER_SIZE
#define ACCEL_BUFFER_SIZE    1024
#endif

/* Bajty sterujące ramki */
#define PROTO_SOF  0x7E
#define PROTO_EOF  0x7C
#define PROTO_ESC  0x7D

/* Adresy urządzeń */
#define ADDR_PC     0x01
#define ADDR_STM32  0x02

/* Lista komend zgodna ze specyfikacją (0x10 - 0x15) */
typedef enum {
  CMD_START       = 0x10, // Interval (1B)
  CMD_STOP        = 0x11, // Brak arg
  CMD_GET_DATA    = 0x12, // Brak arg -> X,Y,Z
  CMD_GET_BUFFER  = 0x13, // Offset(2B), Count(1B)
  CMD_SET_TIME    = 0x14, // Time(4B)
  CMD_GET_ARCHIVE = 0x15, // Back(2B), Count(1B)

  /* Kod błędu (używany tylko w odpowiedzi STM32 -> PC) */
  CMD_ERROR       = 0xFF
} proto_cmd_t;

/* Kody statusu (do CMD_START, CMD_STOP, CMD_SET_TIME) */
typedef enum {
  STATUS_OK   = 0x00,
  STATUS_BUSY = 0x01
} proto_status_t;

/* Kody błędów protokołu */
typedef enum {
  ERR_FRAME_LEN   = 0xE0,
  ERR_CRC         = 0xE1,
  ERR_SEQ         = 0xE2,
  ERR_UNKNOWN_CMD = 0xE3,
  ERR_ARG_RANGE   = 0xE4,
  ERR_BUFFER_FULL = 0xE5
} proto_err_t;

/* Funkcje publiczne */
void Protocol_Init(void);
void Protocol_ParseByte(uint8_t b);
void Protocol_NotifyRxOverflow(void);

/* Callback TX (dla kompatybilności nagłówka, choć teraz wysyłamy przez main.c) */
void Protocol_OnUartTxCplt(void);

#endif /* INC_PROTOCOL_H_ */
