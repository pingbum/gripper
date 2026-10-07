#pragma once
#include <cstdint>

enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
enum GPIO_PinState { GPIO_PIN_RESET, GPIO_PIN_SET };
struct SPI_HandleTypeDef {};
struct GPIO_TypeDef {};
#define __ALIGNED(n) __attribute__((aligned(n)))
#define DRVOFF_GPIO_Port nullptr
#define DRVOFF_Pin 0x10U

extern uint32_t test_primask;
extern uint32_t test_ipsr;
inline uint32_t __get_PRIMASK() { return test_primask; }
inline uint32_t __get_IPSR() { return test_ipsr; }
inline void __disable_irq() { test_primask = 1U; }
inline void __set_PRIMASK(uint32_t value) { test_primask = value; }
inline void __DMB() {}
void HAL_Delay(uint32_t milliseconds);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
