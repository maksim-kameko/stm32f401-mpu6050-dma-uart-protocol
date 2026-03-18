#include "protocol.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>

/* Globalny uchwyt sprzętowy do portu UART (wykorzystywany przy inicjacji transmisji) */
extern UART_HandleTypeDef huart2;

#ifndef USART_TXBUF_LEN
#define USART_TXBUF_LEN 2048
#endif

/* Import zmiennych reprezentujących bufor kołowy transmisji UART */
extern uint8_t USART_TxBuf[];
extern volatile uint16_t USART_TX_Empty; // Wskaźnik zapisu (Głowa)
extern volatile uint16_t USART_TX_Busy;  // Wskaźnik odczytu (Ogon)

/* Podłączenie struktur z logiką pomiarów MPU z main.c */
typedef struct { int16_t x; int16_t y; int16_t z; } accel_sample_t;
extern accel_sample_t accel_buf[];
extern volatile uint16_t accel_head;
extern volatile uint32_t accel_count;

/* Flagi systemowe */
extern volatile uint8_t  g_acq_enabled;
extern volatile uint32_t g_interval_ms;
extern volatile uint32_t g_time_base_ms;

/* ===================== LOGIKA NADAWANIA RAMKI (TX) ===================== */

/* Dodaje jeden bajt na koniec bufora kołowego TX */
static void tx_push_byte(uint8_t b)
{
  uint16_t next = USART_TX_Empty + 1;
  if (next >= USART_TXBUF_LEN) next = 0; // Zawinięcie wskaźnika

  // Zabezpieczenie przed przepełnieniem
  if (next == USART_TX_Busy) return;

  USART_TxBuf[USART_TX_Empty] = b; // Wpis bajtu
  USART_TX_Empty = next;           // Zapis nowej pozycji wskaźnika
}

/* Funkcja wstawiająca znaki ucieczki (Byte Stuffing) */
static void tx_push_escaped(uint8_t b)
{
    // Jeżeli trafimy na znak ramki w danych, wstawiamy ESC i maskujemy go XORem
    if (b == PROTO_SOF || b == PROTO_EOF || b == PROTO_ESC) {
        tx_push_byte(PROTO_ESC);
        tx_push_byte(b ^ 0x20);
    } else {
        tx_push_byte(b); // Jak znak jest normalny, wysyłamy bez zmian
    }
}

/* Wymuszenie startu wysyłki sprzętowej */
static void tx_kick(void)
{
  if (huart2.gState == HAL_UART_STATE_READY)
  {
    if (USART_TX_Empty != USART_TX_Busy)
    {
      uint16_t idx = USART_TX_Busy;
      USART_TX_Busy++;
      if (USART_TX_Busy >= USART_TXBUF_LEN) USART_TX_Busy = 0;

      HAL_UART_Transmit_IT(&huart2, &USART_TxBuf[idx], 1);
    }
  }
}

void Protocol_OnUartTxCplt(void) { }

/* ===================== LOGIKA KONTROLI BŁĘDU CRC-32 ===================== */

static uint32_t crc32_mpeg2(const uint8_t *data, uint16_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint32_t)data[i] << 24;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80000000) crc = (crc << 1) ^ 0x04C11DB7; else crc <<= 1;
    }
  }
  return crc;
}

/* ===================== KOMPOZYCJA ORAZ BUDOWANIE RAMKI WYSYŁAJĄCEJ ===================== */

static void Protocol_SendFrame(uint8_t seq, uint8_t src, uint8_t dst, uint8_t cmd,
                               const uint8_t *payload, uint16_t payload_len)
{
  if (payload_len > 255) payload_len = 255;

  uint8_t logical[4 + 255];
  logical[0] = seq; logical[1] = src; logical[2] = dst; logical[3] = cmd;
  if (payload_len) memcpy(&logical[4], payload, payload_len);

  uint32_t crc = crc32_mpeg2(logical, 4 + payload_len);

  // PAKIETOWANIE I WYSYŁKA
  tx_push_byte(PROTO_SOF); // Stempel STARTU

  // Wysłanie sekcji danych z użyciem funkcji realizującej Byte Stuffing
  for (uint16_t i = 0; i < 4 + payload_len; i++) {
      tx_push_escaped(logical[i]);
  }

  // Wysłanie sumy kontrolnej w formacie Little Endian (też z Byte Stuffingiem)
  for (uint8_t i = 0; i < 4; i++) {
      tx_push_escaped((uint8_t)((crc >> (8*i)) & 0xFF));
  }

  tx_push_byte(PROTO_EOF); // Znak zatwierdzający zakończenie procedury

  tx_kick(); // Otwarcie przepustnicy sprzętowej na nadawanie
}

static void Protocol_SendError(uint8_t seq, uint8_t err) {
  Protocol_SendFrame(seq, ADDR_STM32, ADDR_PC, CMD_ERROR, &err, 1);
}

static void Protocol_SendAckStatus(uint8_t seq, uint8_t cmd, uint8_t status) {
  Protocol_SendFrame(seq, ADDR_STM32, ADDR_PC, cmd, &status, 1);
}

/* ===================== LOGIKA ODBIERANIA DANYCH I PARSOWANIA ===================== */

static enum { RX_IDLE, RX_IN_FRAME } rx_state = RX_IDLE;
static uint8_t rx_escaping = 0;
static uint8_t rx_frame[300];
static uint16_t rx_len = 0;
static uint8_t expected_seq = 0;
static uint8_t seq_synced = 0;

static uint16_t accel_valid_count(void) {
  uint32_t c = accel_count;
  return (c > ACCEL_BUFFER_SIZE) ? ACCEL_BUFFER_SIZE : (uint16_t)c;
}

/* --- OBSŁUGA KOMEND DLA UŻYTKOWNIKA --- */

static void handle_start(uint8_t seq, const uint8_t *data, uint16_t len) {
  if (len != 4) {
      Protocol_SendError(seq, ERR_ARG_RANGE);
      return;
  }

  uint32_t val = (uint32_t)data[0] |
                 ((uint32_t)data[1] << 8) |
                 ((uint32_t)data[2] << 16) |
                 ((uint32_t)data[3] << 24);

  if (val == 0) val = 1;

  g_interval_ms = val;
  g_acq_enabled = 1;
  Protocol_SendAckStatus(seq, CMD_START, STATUS_OK);
}

static void handle_get_data(uint8_t seq) {
  if (accel_valid_count() == 0) { Protocol_SendError(seq, ERR_ARG_RANGE); return; }

  // Brak wyłączania przerwań. Czytanie odbywa się bezpiecznie w pętli głównej (brak kolizji z MPU_ProcessData)
  uint16_t idx = (accel_head + ACCEL_BUFFER_SIZE - 1) % ACCEL_BUFFER_SIZE;
  accel_sample_t s = accel_buf[idx];

  uint8_t out[6];
  out[0] = s.x & 0xFF; out[1] = s.x >> 8;
  out[2] = s.y & 0xFF; out[3] = s.y >> 8;
  out[4] = s.z & 0xFF; out[5] = s.z >> 8;
  Protocol_SendFrame(seq, ADDR_STM32, ADDR_PC, CMD_GET_DATA, out, 6);
}

static void handle_get_buffer(uint8_t seq, const uint8_t *data, uint16_t len) {
  if (len != 3) { Protocol_SendError(seq, ERR_ARG_RANGE); return; }
  uint16_t offset = data[0] | (data[1] << 8);
  uint8_t count = data[2];
  uint16_t valid = accel_valid_count();

  if (count == 0 || count > 42 || offset >= valid || (offset + count) > valid) {
    Protocol_SendError(seq, ERR_ARG_RANGE); return;
  }

  uint8_t out[42*6];

  for(uint8_t i=0; i<count; i++) {
    accel_sample_t s = accel_buf[(offset + i) % ACCEL_BUFFER_SIZE];
    uint16_t k = i*6;
    out[k] = s.x & 0xFF; out[k+1] = s.x >> 8;
    out[k+2] = s.y & 0xFF; out[k+3] = s.y >> 8;
    out[k+4] = s.z & 0xFF; out[k+5] = s.z >> 8;
  }

  Protocol_SendFrame(seq, ADDR_STM32, ADDR_PC, CMD_GET_BUFFER, out, count*6);
}

static void handle_get_archive(uint8_t seq, const uint8_t *data, uint16_t len) {
  if (len != 3) { Protocol_SendError(seq, ERR_ARG_RANGE); return; }
  uint16_t back = data[0] | (data[1] << 8);
  uint8_t count = data[2];
  uint16_t valid = accel_valid_count();

  if (count == 0 || count > 42 || back == 0 || back > valid) {
    Protocol_SendError(seq, ERR_ARG_RANGE); return;
  }

  uint8_t out[42*6];

  int32_t start_idx = (int32_t)accel_head - (int32_t)back;
  if (start_idx < 0) start_idx += ACCEL_BUFFER_SIZE;

  for(uint8_t i=0; i<count; i++) {
    accel_sample_t s = accel_buf[(start_idx + i) % ACCEL_BUFFER_SIZE];
    uint16_t k = i*6;
    out[k] = s.x & 0xFF; out[k+1] = s.x >> 8;
    out[k+2] = s.y & 0xFF; out[k+3] = s.y >> 8;
    out[k+4] = s.z & 0xFF; out[k+5] = s.z >> 8;
  }

  Protocol_SendFrame(seq, ADDR_STM32, ADDR_PC, CMD_GET_ARCHIVE, out, count*6);
}

static void process_frame(void) {
  if (rx_len < 8 || rx_frame[2] != ADDR_STM32) { rx_state = RX_IDLE; return; }

  uint32_t crc_calc = crc32_mpeg2(rx_frame, rx_len - 4);
  uint32_t crc_rx; memcpy(&crc_rx, &rx_frame[rx_len-4], 4);

  if (crc_rx != crc_calc) { Protocol_SendError(rx_frame[0], ERR_CRC); rx_state = RX_IDLE; return; }

  uint8_t seq = rx_frame[0];
  if (!seq_synced) { expected_seq = seq + 1; seq_synced = 1; }
  else if (seq != expected_seq) {
    Protocol_SendError(seq, ERR_SEQ); expected_seq = seq + 1; rx_state = RX_IDLE; return;
  } else expected_seq++;

  uint8_t cmd = rx_frame[3];
  uint8_t *d = &rx_frame[4];
  uint16_t d_len = rx_len - 8;

  switch(cmd) {
    case CMD_START:       handle_start(seq, d, d_len); break;
    case CMD_STOP:        g_acq_enabled = 0; Protocol_SendAckStatus(seq, CMD_STOP, STATUS_OK); break;
    case CMD_GET_DATA:    handle_get_data(seq); break;
    case CMD_GET_BUFFER:  handle_get_buffer(seq, d, d_len); break;
    case CMD_SET_TIME:
               if(d_len==4) {
                 g_time_base_ms = d[0] | (d[1]<<8) | (d[2]<<16) | (d[3]<<24);
                 Protocol_SendAckStatus(seq, CMD_SET_TIME, STATUS_OK);
               } else Protocol_SendError(seq, ERR_ARG_RANGE);
               break;
    case CMD_GET_ARCHIVE: handle_get_archive(seq, d, d_len); break;
    default:   Protocol_SendError(seq, ERR_UNKNOWN_CMD); break;
  }
  rx_state = RX_IDLE;
}

void Protocol_ParseByte(uint8_t b) {

    // Stan 1: Układ śpi i czeka na start
    if (rx_state == RX_IDLE) {
        if (b == PROTO_SOF) {
            rx_state = RX_IN_FRAME; // Zmieniamy stan na "Czytam"
            rx_len = 0;
            rx_escaping = 0;
        }
    }
    // Stan 2: Układ jest w trakcie odbierania danych
    else if (rx_state == RX_IN_FRAME) {

        if (b == PROTO_SOF) {
            // Zaczynamy od nowa (błąd PC)
            rx_len = 0;
            rx_escaping = 0;
        }
        else if (b == PROTO_EOF) {
            // Koniec ramki, idziemy przetwarzać
            process_frame();
        }
        else if (b == PROTO_ESC) {
            // Znak ucieczki, zapalamy flagę
            rx_escaping = 1;
        }
        else {
            // Zwykły znak danych pomiarowych

            // Odkodowanie (Byte Destuffing)
            if (rx_escaping) {
                b ^= 0x20;
                rx_escaping = 0;
            }

            // Zapisz znak do bufora ramki
            if (rx_len < sizeof(rx_frame)) {
                rx_frame[rx_len++] = b;
            } else {
                // Przepełnienie bufora
                Protocol_SendError(0, ERR_FRAME_LEN);
                rx_state = RX_IDLE;
            }
        }
    }
}

void Protocol_Init(void) {
  rx_state = RX_IDLE; seq_synced = 0;
  USART_TX_Empty = 0; USART_TX_Busy = 0;
}

void Protocol_NotifyRxOverflow(void) { Protocol_SendError(0, ERR_BUFFER_FULL); }
