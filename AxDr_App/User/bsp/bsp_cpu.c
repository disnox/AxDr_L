#include "bsp_cpu.h"
#include "bsp.h"

void bsp_cpu_reset(void)
{
    NVIC_SystemReset();
}

void bsp_cpu_get_uid(uint32_t uid[3])
{
	uid[0] = *(uint32_t*)(ADDR_UID);
	uid[1] = *(uint32_t*)(ADDR_UID+4);
	uid[2] = *(uint32_t*)(ADDR_UID+8);
}

void bsp_cpu_get_chip_type(uint32_t *chip_type)
{
	*chip_type = *(uint32_t*)(ADDR_MCU_TYPE);	//MCU ID
}
























