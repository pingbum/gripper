#ifndef INC_FLASH_MEMORY_HPP_
#define INC_FLASH_MEMORY_HPP_

#include "main.h"
#include "arm_math_types.h"
#define PAGE_IDX(addr) (((addr) - 0x08000000UL) / FLASH_PAGE_SIZE) % 128
#define PAGE_BANK(addr) (((addr) < 0x08040000UL) ? FLASH_BANK_1 : FLASH_BANK_2)
#define FLASH_BANK_2_ADDR 0x08040000UL

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Write a page to flash memory.
 * @param addr Address to write to.
 * @param src Pointer to the source data.
 * @param n Number of bytes to write.
 * @return HAL status code.
 */
__attribute__((section(".ramfunc")))
uint32_t write_flash_page(uint32_t addr, uint8_t *src, uint32_t n);

/**
 * @brief Write a buffer to flash memory.
 * @param addr Base address to write to.
 * @param src Pointer to the source buffer.
 * @param length Length of the source buffer in elements.
 * @param size Size of each element in bytes.
 * @return HAL status code.
 */
__attribute__((section(".ramfunc")))
uint32_t write_flash_buffer(uint32_t addr, uint8_t *src, uint32_t length, uint32_t size);

/**
 * @brief Read a float buffer from flash memory.
 * @param addr Base address to read from.
 * @param dst Destination buffer.
 * @param length Number of elements.
 * @param size Size of each element in bytes.
 * @return HAL status code.
 */
uint32_t read_flash_buffer_f32(uint32_t addr, float32_t *dst, uint32_t length, uint32_t size);
#ifdef __cplusplus
}
#endif

#endif /* INC_FLASH_MEMORY_HPP_ */
