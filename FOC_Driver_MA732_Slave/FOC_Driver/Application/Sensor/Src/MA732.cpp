#include "MA732.hpp"
#include "spi.h"
#include "tim.h"
#include "main.h"
#include "error_handler.hpp"

namespace {
constexpr uint32_t TIMEOUT_MS = 5U;
alignas(4) volatile uint16_t rx_words[2];
volatile MA732_t::Sample latest{};
volatile uint32_t last_tick = 0;
volatile bool started = false;
bool armed = false;
uint32_t cr1 = 0, cr2 = 0;

// Called only while CS is high. Reset also discards a partial shift register.
// DMA has room for two words so an extra complete word is rejected at CS rise.
void arm_next_frame() {
  SPI3->CR1 &= ~SPI_CR1_SPE;
  DMA2_Channel2->CCR &= ~DMA_CCR_EN;
  __HAL_RCC_SPI3_FORCE_RESET();
  __HAL_RCC_SPI3_RELEASE_RESET();
  SPI3->CR1 = cr1; // SSM=1, SSI=0: selected ahead of the first clock.
  SPI3->CR2 = cr2;
  DMA2->IFCR = DMA_IFCR_CGIF2;
  DMA2_Channel2->CPAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&SPI3->DR));
  DMA2_Channel2->CMAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(rx_words));
  DMA2_Channel2->CNDTR = 2;
  DMA2_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_PSIZE_0 |
                       DMA_CCR_MSIZE_0 | DMA_CCR_PL_1 | DMA_CCR_PL_0;
  DMA2_Channel2->CCR |= DMA_CCR_EN;
  SPI3->CR2 |= SPI_CR2_RXDMAEN;
  SPI3->CR1 |= SPI_CR1_SPE;
  armed = (GPIOB->IDR & GPIO_PIN_6) != 0U;
}
}

HAL_StatusTypeDef MA732_t::start() {
  if (hspi3.Init.Mode != SPI_MODE_SLAVE ||
      hspi3.Init.Direction != SPI_DIRECTION_2LINES_RXONLY ||
      hspi3.Init.NSS != SPI_NSS_SOFT ||
      hspi3.Init.DataSize != SPI_DATASIZE_16BIT)
    return HAL_ERROR;
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  armed = false;
  latest.sequence = 0;
  last_tick = 0;
  HAL_NVIC_DisableIRQ(DMA2_Channel2_IRQn); // DMA flags are sampled at CS rise.
  cr1 = SPI3->CR1 & ~(SPI_CR1_SPE | SPI_CR1_SSI);
  cr2 = SPI3->CR2 & ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN |
                       SPI_CR2_RXNEIE | SPI_CR2_TXEIE | SPI_CR2_ERRIE);
  GPIO_InitTypeDef gpio{};
  gpio.Pin = GPIO_PIN_6;
  gpio.Mode = GPIO_MODE_IT_RISING;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpio);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
  started = true;
  if ((GPIOB->IDR & GPIO_PIN_6) != 0U)
    arm_next_frame();
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  return HAL_OK;
}

extern "C" void EXTI9_5_IRQHandler(void) {
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) == 0U)
    return;
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
  const uint32_t cycles = TIM2->CNT;
  // If the next frame has already started, discard and wait for its end.
  const bool cs_high = (GPIOB->IDR & GPIO_PIN_6) != 0U;
  DMA2_Channel2->CCR &= ~DMA_CCR_EN;
  __DSB();
  const bool valid = armed && cs_high && DMA2_Channel2->CNDTR == 1U &&
      (DMA2->ISR & DMA_ISR_TEIF2) == 0U &&
      (SPI3->SR & (SPI_SR_OVR | SPI_SR_RXNE | SPI_SR_MODF)) == 0U;
  if (valid) {
    latest.word = rx_words[0];
    latest.cycles = cycles;
    last_tick = HAL_GetTick();
    latest.sequence = latest.sequence + 1U;
  }
  armed = false;
  SPI3->CR1 &= ~SPI_CR1_SPE;
  if (cs_high)
    arm_next_frame();
}

HAL_StatusTypeDef MA732_t::readSample(Sample &sample) {
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  sample.word = latest.word;
  sample.cycles = latest.cycles;
  sample.sequence = latest.sequence;
  const uint32_t tick = last_tick;
  __set_PRIMASK(mask);
  if (sample.sequence == 0U)
    return HAL_BUSY;
  return HAL_GetTick() - tick >= TIMEOUT_MS ? HAL_TIMEOUT : HAL_OK;
}
HAL_StatusTypeDef MA732_t::readAngle(uint16_t &angle) {
  Sample sample{};
  const auto status = readSample(sample);
  if (status == HAL_OK) angle = sample.word;
  return status;
}
uint16_t MA732_t::readAngleRaw() {
  uint16_t word = 0;
  if (readAngle(word) != HAL_OK) Error_Raise(ERROR_SPI);
  return word >> 2U;
}
float MA732_t::readAngleDeg() {
  return readAngleRaw() * (360.0f / 16384.0f);
}
extern "C" void MA732_SlaveWatchdog(void) {
  if (started && latest.sequence != 0U &&
      HAL_GetTick() - last_tick >= TIMEOUT_MS) {
    // Immediate gate disable also covers blocking calibration and flash work.
    DRVOFF_GPIO_Port->BSRR = DRVOFF_Pin;
    Error_SetFlags(ERROR_SPI);
  }
}
