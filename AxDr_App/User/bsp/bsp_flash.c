#include "bsp_flash.h"
#include "bsp.h"


uint16_t bsp_get_flash_size(void)
{
	uint16_t flash_size = 0;
	flash_size = *(uint16_t *)(ADDR_FLASH_SIZE);
	return flash_size;
}


uint16_t bsp_get_page(uint32_t addr)
{
    return (addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}

int8_t bsp_flash_erase(uint32_t addr, uint32_t size)
{
	int8_t ret=0;

	HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_WRPERR);

    uint32_t first_page = bsp_get_page(addr);
    uint32_t nbof_page  = bsp_get_page(addr+size) - first_page;

    /* Fill EraseInit structure*/
    FLASH_EraseInitTypeDef FlashSet;
    FlashSet.TypeErase   = FLASH_TYPEERASE_PAGES;
    FlashSet.Page        = first_page;
    FlashSet.NbPages     = nbof_page;

    uint32_t page_err = 0;
    if (HAL_FLASHEx_Erase(&FlashSet, &page_err) != HAL_OK)
        ret = 0;

    HAL_FLASH_Lock();
	
	return ret;
}

int8_t bsp_flash_write(uint32_t addr, const uint32_t *buf, uint32_t size)
{
    int ret = 0;

    if (size == 0)
        return 0;

    if (buf == NULL)
        return -1;

    ret = bsp_flash_erase(addr, size);
    if (ret != 0)
        return -2;

    ret = HAL_FLASH_Unlock();
    if (ret != HAL_OK)
        return -3;

    int write8ByteCnt = size % 8;
    if (write8ByteCnt == 0)
    {
        write8ByteCnt = size / 8;
    }
    else
    {
        write8ByteCnt = size / 8 + 1;
    }

    for (int i = 0; i < write8ByteCnt; i++)
    {
        uint64_t dataL = *(buf + 2 * i);
        uint64_t dataH = *(buf + 2 * i + 1);
        uint64_t data = (dataH << 32) | dataL;

        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
        ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + 8 * i, data);
        if (ret != HAL_OK)
        {
            HAL_FLASH_Lock();
            return -3;
        }
    }

    ret = HAL_FLASH_Lock();
    if (ret != HAL_OK)
        return -4;

    return ret;
}

void bsp_flash_read(uint32_t addr ,uint32_t *buf,uint32_t size)
{
	memcpy(buf, (uint32_t *)addr, size);
}























