/**
 * @file main_cpp.hpp
 * @brief Application entry point and callback declarations.
 */

#pragma once

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief C++ application entry point.
 */
void main_cpp(void);

/**
 * @brief ADC conversion complete callback (injected conversions).
 */
void HAL_ADC_ConversionENDCallback(ADC_HandleTypeDef *hadc);

/**
 * @brief TIM6 periodic callback.
 */
void TIM6_PeriodElapsedCB(TIM_HandleTypeDef *htim);

/**
 * @brief TIM7 periodic callback.
 */
void TIM7_PeriodElapsedCB(TIM_HandleTypeDef *htim);

/**
 * @brief TIM17 periodic callback.
 */
void TIM17_PeriodElapsedCB(TIM_HandleTypeDef *htim);

/**
 * @brief TIM16 periodic callback.
 */
void TIM16_PeriodElapsedCB(TIM_HandleTypeDef *htim);

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc5;

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;

extern HRTIM_HandleTypeDef hhrtim1;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

extern FDCAN_HandleTypeDef hfdcan1;
#ifdef __cplusplus
}
#endif
