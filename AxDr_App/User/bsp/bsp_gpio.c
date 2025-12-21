#include "bsp_gpio.h"
#include "bsp.h"

const GPIO_TypeDef *Stm32PortList[MCU_PORT_MAX] = {NULL, GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG};



void bsp_led_init(void)
{

}

void bsp_key_init(void)
{

}

void bsp_gpio_write_bit(bsp_port_e port, bsp_io_e pin, io_sta_e io_sta)
{
	if(port == NULL)
		return;
	HAL_GPIO_WritePin((GPIO_TypeDef *)Stm32PortList[port], pin, (GPIO_PinState)io_sta);
}

void bsp_gpio_toggle_bit(bsp_port_e port, bsp_io_e pin)
{
	if(port == NULL)
		return;
	HAL_GPIO_TogglePin((GPIO_TypeDef *)Stm32PortList[port], pin);
}
            

int8_t bsp_gpio_read_bit(bsp_port_e port, bsp_io_e pin)
{
	int8_t ret = 0;
	
	if(port == NULL)
		return IO_STA_1;
	
	HAL_GPIO_ReadPin((GPIO_TypeDef *)Stm32PortList[port], pin);
	return ret;
}

uint32_t get_bsp_tick(void)
{
	return HAL_GetTick();
}


