#pragma once
#include <cstdint>
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
struct Spi { uint32_t CR1=0, CR2=0, DR=0, SR=0; };
struct DmaChannel { uint32_t CCR=0, CPAR=0, CMAR=0, CNDTR=0; };
struct Dma { uint32_t IFCR=0, ISR=0; };
struct Gpio { uint32_t IDR=0, BSRR=0; };
struct Timer { uint32_t CNT=0; };
struct SpiInit { uint32_t Mode, Direction, NSS, DataSize; };
struct SpiHandle { SpiInit Init; };
struct GPIO_InitTypeDef { uint32_t Pin=0, Mode=0, Pull=0; };
inline Spi spi;
inline Dma dma;
inline DmaChannel channel;
inline Gpio gpio;
inline Timer timer;
inline Spi *SPI3 = &spi;
inline Dma *DMA2 = &dma;
inline DmaChannel *DMA2_Channel2 = &channel;
inline Gpio *GPIOB = &gpio;
inline Timer *TIM2 = &timer;
inline Gpio *DRVOFF_GPIO_Port = &gpio;
constexpr uint32_t DRVOFF_Pin=1U<<7, GPIO_PIN_6=1U<<6;
constexpr uint32_t SPI_CR1_SPE=1U<<6, SPI_CR1_SSI=1U<<8;
constexpr uint32_t SPI_CR2_RXDMAEN=1, SPI_CR2_TXDMAEN=2;
constexpr uint32_t SPI_CR2_RXNEIE=1U<<6, SPI_CR2_TXEIE=1U<<7, SPI_CR2_ERRIE=1U<<5;
constexpr uint32_t SPI_SR_RXNE=1, SPI_SR_OVR=1U<<6, SPI_SR_MODF=1U<<5;
constexpr uint32_t DMA_CCR_EN=1, DMA_CCR_MINC=1U<<7, DMA_CCR_PSIZE_0=1U<<8;
constexpr uint32_t DMA_CCR_MSIZE_0=1U<<10, DMA_CCR_PL_1=1U<<13, DMA_CCR_PL_0=1U<<12;
constexpr uint32_t DMA_IFCR_CGIF2=1U<<4, DMA_ISR_TEIF2=1U<<7;
constexpr uint32_t SPI_MODE_SLAVE=0, SPI_DIRECTION_2LINES_RXONLY=1U<<10;
constexpr uint32_t SPI_NSS_SOFT=1U<<9, SPI_DATASIZE_16BIT=15U<<8;
constexpr uint32_t GPIO_MODE_IT_RISING=1, GPIO_PULLUP=1;
constexpr int EXTI9_5_IRQn=23, DMA2_Channel2_IRQn=57;
inline SpiHandle hspi3{{SPI_MODE_SLAVE,SPI_DIRECTION_2LINES_RXONLY,SPI_NSS_SOFT,SPI_DATASIZE_16BIT}};
inline uint32_t tick=0, primask=0, pending=0, errors=0;
inline uint32_t HAL_GetTick() { return tick; }
inline uint32_t __get_PRIMASK() { return primask; }
inline void __disable_irq() { primask=1; }
inline void __set_PRIMASK(uint32_t value) { primask=value; }
inline void __DSB() {}
inline void HAL_NVIC_DisableIRQ(int) {}
inline void HAL_NVIC_EnableIRQ(int) {}
inline void HAL_NVIC_SetPriority(int,int,int) {}
inline void HAL_GPIO_Init(Gpio*, GPIO_InitTypeDef*) {}
inline void __HAL_RCC_SPI3_FORCE_RESET() { spi={}; }
inline void __HAL_RCC_SPI3_RELEASE_RESET() {}
inline void __HAL_GPIO_EXTI_CLEAR_IT(uint32_t pin) { pending &= ~pin; }
inline uint32_t __HAL_GPIO_EXTI_GET_IT(uint32_t pin) { return pending & pin; }
constexpr uint8_t ERROR_SPI=2;
inline void Error_SetFlags(uint8_t flags) { errors |= flags; }
inline void Error_Raise(uint8_t flags) { errors |= flags; }
