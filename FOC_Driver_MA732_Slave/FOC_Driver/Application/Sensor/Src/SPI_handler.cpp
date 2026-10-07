/**
 * @file SPI_handler.cpp
 * @brief SPI DMA helper implementation.
 */

#include "SPI_handler.hpp"
#include "arm_math.h"

// ---- Linked list of active handlers ----
SPIHandler_t *SPIHandler_t::s_head = nullptr;
uint8_t SPIHandler_t::s_txBuf[SPI_HANDLER_MAX_WORDS * 2] __ALIGNED(4);
uint8_t SPIHandler_t::s_rxBuf[SPI_HANDLER_MAX_WORDS * 2] __ALIGNED(4);

SPIHandler_t::SPIHandler_t(SPI_HandleTypeDef *hspi,
                           GPIO_TypeDef *csPort,
                           uint16_t csPin)
    : m_hspi{hspi},
      m_csPort{csPort},
      m_csPin{csPin},
      m_next{s_head}
{
    s_head = this; // Register in the list.
}

// ---- Transfer ----
HAL_StatusTypeDef SPIHandler_t::transfer(uint8_t *tx,
                                         uint8_t *rx,
                                         size_t len)
{
    if (len > SPI_HANDLER_MAX_WORDS)
        return HAL_ERROR;

    // Copy to aligned DMA buffer using CMSIS-DSP helper.
    arm_copy_q7(reinterpret_cast<q7_t *>(tx), reinterpret_cast<q7_t *>(s_txBuf), len * 2);

    csLow();

    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(m_hspi, s_txBuf, s_rxBuf, len);
    if (st != HAL_OK)
    {
        csHigh();
        return st;
    }

    while (HAL_SPI_GetState(m_hspi) != HAL_SPI_STATE_READY)
    {
        /* Wait for DMA completion. */
    }

    arm_copy_q7(reinterpret_cast<q7_t *>(s_rxBuf), reinterpret_cast<q7_t *>(rx), len * 2);

    csHigh();
    return HAL_OK;
}

// ---- IRQ dispatch (DMA complete callback) ----
void SPIHandler_t::irqHandler(SPI_HandleTypeDef *hspi)
{
    for (SPIHandler_t *p = s_head; p; p = p->m_next)
    {
        if (p->m_hspi == hspi)
        {
            p->csHigh();
            break;
        }
    }
}

// ---- HAL weak callback override ----
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPIHandler_t::irqHandler(hspi);
}
