#ifndef __BSP_FLASH_H__
#define __BSP_FLASH_H__

#include "bsp.h"

#define ADDR_FLASH            	(0x08000000UL) 			/*!< FLASH (up to 512 kB) base address */
#define ADDR_FLASH_SIZE        	(0x1FFF75E0UL)        	/*!< Flash size data register base address  */


#define ADDR_FLASH_PAGE_60    ((uint32_t)0x0801E000) /* Base @ of Page 60, 2 Kbytes */
#define ADDR_FLASH_PAGE_61    ((uint32_t)0x0801E800) /* Base @ of Page 61, 2 Kbytes */
#define ADDR_FLASH_PAGE_62    ((uint32_t)0x0801F000) /* Base @ of Page 62, 2 Kbytes */
#define ADDR_FLASH_PAGE_63    ((uint32_t)0x0801F800) /* Base @ of Page 63, 2 Kbytes */

uint16_t bsp_get_flash_size(void);
uint16_t bsp_get_page(uint32_t addr);
int8_t bsp_flash_erase(uint32_t addr, uint32_t size);
int8_t bsp_flash_write(uint32_t addr, const uint32_t *buf, uint32_t size);
void bsp_flash_read(uint32_t addr, uint32_t *buf, uint32_t size);



#endif /* __BSP_FLASH_H__ */


