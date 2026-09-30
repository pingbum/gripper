/**
 * @file SPI_handler.hpp
 * @brief SPI DMA helper with shared aligned buffers.
 */

#ifndef INC_SPI_HANDLER_HPP_
#define INC_SPI_HANDLER_HPP_

#include "main.h"
#include <cstddef>

// Maximum SPI DMA transfer size in 16-bit words.
#ifndef SPI_HANDLER_MAX_WORDS
#define SPI_HANDLER_MAX_WORDS 32
#endif
typedef union
{
    uint16_t data;
    uint8_t packet[2];
} buffer_t;

class SPIHandler_t
{
public:
    /**
     * @brief Create a handler for an SPI peripheral and chip-select pin.
     */
    SPIHandler_t(SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *csPort,
                 uint16_t csPin);

    /**
     * @brief Transfer a block of 16-bit words over SPI using DMA.
     * @param tx Transmit buffer (length in 16-bit words).
     * @param rx Receive buffer (length in 16-bit words).
     * @param len Number of 16-bit words.
     * @return HAL status code.
     */
    HAL_StatusTypeDef transfer(uint8_t *tx,
                               uint8_t *rx,
                               size_t len);

    /**
     * @brief Dispatch DMA completion to the matching handler instance.
     */
    static void irqHandler(SPI_HandleTypeDef *hspi);

private:
    SPI_HandleTypeDef *m_hspi;
    GPIO_TypeDef *m_csPort;
    uint16_t m_csPin;
    static SPIHandler_t *s_head;
    SPIHandler_t *m_next;

    // Aligned DMA buffers shared across instances.
    static uint8_t s_txBuf[SPI_HANDLER_MAX_WORDS * 2] __ALIGNED(4);
    static uint8_t s_rxBuf[SPI_HANDLER_MAX_WORDS * 2] __ALIGNED(4);

    inline void csLow() { HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET); }
    inline void csHigh() { HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET); }
};

#endif /* INC_SPI_HANDLER_HPP_ */
