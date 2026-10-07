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
    if (tx == nullptr || rx == nullptr || len == 0U || len > SPI_HANDLER_MAX_WORDS)
        return HAL_ERROR;

    // A failed peripheral stays faulted until explicitly reinitialized/reset.
    if (HAL_SPI_GetState(m_hspi) != HAL_SPI_STATE_READY)
        return HAL_BUSY;

    // Copy to aligned DMA buffer using CMSIS-DSP helper.
    arm_copy_q7(reinterpret_cast<q7_t *>(tx), reinterpret_cast<q7_t *>(s_txBuf), len * 2);

    csLow();

    const uint32_t start_tick = HAL_GetTick();
    // Secondary finite bound when SysTick cannot preempt the calling ISR.
    // This is an iteration cap, not a calibrated microsecond timeout.
    uint32_t spins_left = SystemCoreClock / 1000U + 1U;
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(m_hspi, s_txBuf, s_rxBuf, len);
    if (st == HAL_OK)
    {
        while (HAL_SPI_GetState(m_hspi) != HAL_SPI_STATE_READY)
        {
            if (m_hspi->ErrorCode != HAL_SPI_ERROR_NONE)
            {
                st = HAL_ERROR;
                break;
            }
            if ((uint32_t)(HAL_GetTick() - start_tick) >= 2U || --spins_left == 0U)
            {
                st = HAL_TIMEOUT;
                break;
            }
        }
        if (st == HAL_OK && m_hspi->ErrorCode != HAL_SPI_ERROR_NONE)
            st = HAL_ERROR;
    }

    if (st != HAL_OK)
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        // Fail closed: no HAL_SPI_Abort polling on BSY/FIFO with a stalled tick.
        CLEAR_BIT(m_hspi->Instance->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN |
                  SPI_CR2_RXNEIE | SPI_CR2_TXEIE | SPI_CR2_ERRIE);
        __HAL_SPI_DISABLE(m_hspi);
        csHigh();
        if (m_hspi->hdmarx && m_hspi->hdmarx->State == HAL_DMA_STATE_BUSY)
            (void)HAL_DMA_Abort(m_hspi->hdmarx);
        if (m_hspi->hdmatx && m_hspi->hdmatx->State == HAL_DMA_STATE_BUSY)
            (void)HAL_DMA_Abort(m_hspi->hdmatx);
        m_hspi->State = HAL_SPI_STATE_ERROR;
        __set_PRIMASK(primask);
        return st; // Never publish a partial/stale receive buffer.
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
