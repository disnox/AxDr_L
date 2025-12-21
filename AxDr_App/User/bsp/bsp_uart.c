#include "bsp_uart.h"
#include "bsp.h"
#include "usart.h"

void bsp_uart_init(void)
{
	
}


void bsp_uart_start(uart_num_e comport)
{

}


int8_t bsp_uart_port_is(uart_num_e comport, UART_HandleTypeDef *huart)
{
	if(comport > fdcan_max) {
		return -1;
	}
	else if(comport == uart1) {
//		huart = &huart1;
	}
	else if(comport == uart2) {
//		huart = &huart2;
	}
	else if(comport == uart3) {
		*huart = huart3;
	}
	else {
		return -1;
	}
	return 0;
}

int8_t bsp_uart_set_baud(uart_num_e comport, uint32_t baud)
{
	UART_HandleTypeDef huart;
	int8_t ret = 0;
	
	if(bsp_uart_port_is(comport, &huart)==0)	
	{
		HAL_UART_DeInit(&huart);
		huart.Init.BaudRate = baud;
		ret = HAL_UART_Init(&huart);
	}
	
    return ret;
}


int8_t bsp_uart_send(uart_num_e comport, uint8_t *pbuf, uint16_t len)
{
	int8_t ret;
	UART_HandleTypeDef huart;
	
	if(bsp_uart_port_is(comport, &huart)==0)	
	{
		ret = HAL_UART_Transmit(&huart, pbuf, len, 0xFFFF);
	}
	
	return ret;
}

int8_t bsp_uart_send_dma(uart_num_e comport, uint8_t *pbuf, uint16_t len)
{
	UART_HandleTypeDef huart;
	int8_t ret = 0;
	
	if(bsp_uart_port_is(comport, &huart)==0)	
	{
		ret = HAL_UART_Transmit_DMA(&huart, pbuf, len);
	}
	
	return ret;
}

int8_t bsp_uart_tx_state(uart_num_e comport)
{
	UART_HandleTypeDef huart;
	int8_t ret = 0;
	
	if(bsp_uart_port_is(comport, &huart)==0)	
	{
		ret = !(huart.gState == HAL_UART_STATE_READY);
	}
	return ret;
}

int8_t bsp_uart_rx_idle_dma(uart_num_e comport, uint8_t *buf, uint16_t len)
{
	UART_HandleTypeDef huart;
	int8_t ret = 0;
	
	if(bsp_uart_port_is(comport, &huart)==0)	
	{
		ret = HAL_UARTEx_ReceiveToIdle_DMA(&huart, buf, len);
	}
	return ret;
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3)
	{
		uart_tx_callback(uart3);	
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart == &huart3)
	{
		uart_rx_callback(uart3, Size);	
	}
}

__weak void uart_tx_callback(uart_num_e comport)
{

}

__weak void uart_rx_callback(uart_num_e comport, uint16_t len)
{

}


