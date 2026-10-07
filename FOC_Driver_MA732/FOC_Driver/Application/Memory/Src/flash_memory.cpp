#include "flash_memory.hpp"
#include <cstring>

__attribute__((section(".ramfunc")))
uint32_t
write_flash_page(uint32_t addr, uint8_t *src, uint32_t n)
{
    __disable_irq();
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    FLASH_EraseInitTypeDef e = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks = PAGE_BANK(addr),
        .Page = PAGE_IDX(addr),
        .NbPages = 1};

    uint32_t pe;
    if (HAL_FLASHEx_Erase(&e, &pe) != HAL_OK)
    {
        HAL_FLASH_Lock();
        __enable_irq();
        return HAL_FLASH_GetError();
    }

    for (uint32_t i = 0; i < n; i += 8)
    {
        uint64_t d = 0xFFFFFFFFFFFFFFFFULL;
        memcpy(&d, src + i, (n - i) >= 8 ? 8 : (n - i));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              addr + i, d) != HAL_OK)
        {
            HAL_FLASH_Lock();
            __enable_irq();
            return HAL_FLASH_GetError();
        }
    }

    HAL_FLASH_Lock();
    __enable_irq();
    return HAL_FLASH_GetError();
}

__attribute__((section(".ramfunc")))
uint32_t
write_flash_buffer(uint32_t addr, uint8_t *src, uint32_t length, uint32_t size)
{
    uint32_t totalBytes = length * size;
    __disable_irq();
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    uint32_t pStart = addr;
    uint32_t pEnd = addr + totalBytes;
    uint32_t num_page = PAGE_IDX(pEnd) - PAGE_IDX(pStart) + 1;

    // Erase data area.
    FLASH_EraseInitTypeDef eraseData = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks = PAGE_BANK(pStart),
        .Page = PAGE_IDX(pStart),
        .NbPages = num_page};

    uint32_t pe;
    if (HAL_FLASHEx_Erase(&eraseData, &pe) != HAL_OK)
    {
        HAL_FLASH_Lock();
        __enable_irq();
        return HAL_FLASH_GetError();
    }

    for (uint32_t offset = 0; offset < totalBytes; offset += 8)
    {
        uint64_t d = 0xFFFFFFFFFFFFFFFFULL;
        uint32_t chunk = (totalBytes - offset) >= 8 ? 8 : (totalBytes - offset);
        memcpy(&d, src + offset, chunk);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              pStart + offset, d) != HAL_OK)
        {
            HAL_FLASH_Lock();
            __enable_irq();
            return HAL_FLASH_GetError();
        }
    }

    HAL_FLASH_Lock();
    __enable_irq();
    return HAL_FLASH_GetError();
}

uint32_t read_flash_buffer_f32(uint32_t addr, float32_t *dst, uint32_t length, uint32_t size)
{
    // Flash is memory-mapped; memcpy reads directly.
    uint32_t pStart = addr;
    memcpy(dst, (const void *)pStart, length * size);
    return HAL_FLASH_GetError();
}

uint32_t read_flash_page(uint32_t addr, uint8_t *dst, uint32_t n)
{
    // Flash is memory-mapped; memcpy reads directly.
    uint8_t *src = (uint8_t *)addr;
    memcpy(dst, src, n);
    return HAL_FLASH_GetError();
}
