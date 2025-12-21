#ifndef __BSP_CPU_H__
#define __BSP_CPU_H__

#include "bsp.h"

//0x1FFFF7AC,  /*STM32F0唯一ID起始地址*/
//0x1FFFF7E8,  /*STM32F1唯一ID起始地址*/
//0x1FFF7A10,  /*STM32F2唯一ID起始地址*/
//0x1FFFF7AC,  /*STM32F3唯一ID起始地址*/
//0x1FFF7A10,  /*STM32F4唯一ID起始地址*/
//0x1FF0F420,  /*STM32F7唯一ID起始地址*/
//0x1FF80050,  /*STM32L0唯一ID起始地址*/
//0x1FF80050,  /*STM32L1唯一ID起始地址*/
//0x1FFF7590,  /*STM32L4唯一ID起始地址*/
//0x1FFF7590,  /*STM32G4唯一ID起始地址*/
//0x1FF0F420}; /*STM32H7唯一ID起始地址*/



#define ADDR_UID 			(0x1FFF7590UL)
#define ADDR_MCU_TYPE       (0xE0042000UL)

void bsp_cpu_get_uid(uint32_t uid[3]);
void bsp_cpu_get_chip_type(uint32_t *chip_type);
void bsp_cpu_reset(void);

#endif  /* __BSP_CPU_H__ */



















