#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "bsp.h"


typedef enum {
  uart1 =0,
  uart2,
  uart3,
  uart_max,
}uart_num_e; 

void bsp_uart_init(void);
int8_t bsp_uart_set_baud(uart_num_e comport, uint32_t baud);
int8_t bsp_uart_send(uart_num_e comport, uint8_t *pbuf, uint16_t len);
int8_t bsp_uart_send_dma(uart_num_e comport, uint8_t *pbuf, uint16_t len);
int8_t bsp_uart_tx_state(uart_num_e comport);
int8_t bsp_uart_rx_idle_dma(uart_num_e comport, uint8_t *buf, uint16_t len);
// __waek
void uart_tx_callback(uart_num_e comport);
void uart_rx_callback(uart_num_e comport, uint16_t len);

#endif /* __BSP_UART_H__ */

