#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

#include "bsp.h"


/* led io */
#define LED_R_PORT  MCU_PORT_C
#define LED_R_PIN   MCU_IO_14

#define LED_G_PORT  MCU_PORT_C
#define LED_G_PIN   MCU_IO_13

#define LED_B_PORT  MCU_PORT_C
#define LED_B_PIN   MCU_IO_15


/* key io */
#define key_1 bsp_gpio_read_bit(MCU_PORT_C, MCU_IO_6)
#define key_2 bsp_gpio_read_bit(MCU_PORT_C, MCU_IO_7)
#define key_3 bsp_gpio_read_bit(MCU_PORT_C, MCU_IO_8)
#define key_4 bsp_gpio_read_bit(MCU_PORT_C, MCU_IO_9)


/* io端口组名称转义*/
typedef enum{
	MCU_PORT_NULL = 0x00,
	MCU_PORT_A,
	MCU_PORT_B,
	MCU_PORT_C,
	MCU_PORT_D,
	MCU_PORT_E,
	MCU_PORT_F,
	MCU_PORT_G,
	MCU_PORT_H,
	MCU_PORT_I,
	MCU_PORT_J,
	MCU_PORT_K,
	MCU_PORT_MAX
}bsp_port_e;

/*IO位定义，每个PORT支持16个BIT*/
typedef enum{
	MCU_IO_0 = 0x0001,
	MCU_IO_1 = 0x0002,
	MCU_IO_2 = 0x0004,
	MCU_IO_3 = 0x0008,
	MCU_IO_4 = 0x0010,
	MCU_IO_5 = 0x0020,
	MCU_IO_6 = 0x0040,
	MCU_IO_7 = 0x0080,
	MCU_IO_8 = 0x0100,
	MCU_IO_9 = 0x0200,
	MCU_IO_10 = 0x0400,
	MCU_IO_11 = 0x0800,
	MCU_IO_12 = 0x1000,
	MCU_IO_13 = 0x2000,
	MCU_IO_14 = 0x4000,
	MCU_IO_15 = 0x8000,
	MCU_IO_MAX = 16,
}bsp_io_e;

/*io state */
typedef enum
{
	IO_STA_0 = 0x00,
	IO_STA_1 = 0X01,
}io_sta_e;


#define bsp_gpio_t  GPIO_TypeDef

uint32_t get_bsp_tick(void);

void bsp_led_init(void);
void bsp_key_init(void);

void bsp_gpio_write_bit(bsp_port_e port, bsp_io_e pin, io_sta_e io_sta);
void bsp_gpio_toggle_bit(bsp_port_e port, bsp_io_e pin);
int8_t bsp_gpio_read_bit(bsp_port_e port, bsp_io_e pin);


#endif /* __BSP_GPIO_H__ */

