/*
 * stm32f44xx_adc.h
 *
 *  Created on: 7 Sep 2025
 *      Author: katog
 */

#ifndef INC_STM32F44XX_ADC_H_
#define INC_STM32F44XX_ADC_H_

#include "stm32f4xx.h"
#include "stm32f446xx.h"

typedef enum {
	ADC_RES_12BIT = 0,
	ADC_RES_10BIT,
	ADC_RES_8BIT,
	ADC_RES_6BIT

}ADC_Resolution_t;

typedef enum {
	ADC_SINGLE_CONVERSION_MODE = 0,
	ADC_CONTINUOUS_CONVERSION_MODE

}ADC_Conversion_Mode_t;

typedef enum {
	ADC_DATA_RIGHT_ALIGNED = 0,
	ADC_DATA_LEFT_ALIGNED

}ADC_Data_Alignment_t;

typedef enum {
	ADC_3_CYCLES = 0,
	ADC_15_CYCLES,
	ADC_28_CYCLES,
	ADC_56_CYCLES,
	ADC_84_CYCLES,
	ADC_112_CYCLES,
	ADC_144_CYCLES,
	ADC_480_CYCLES

}ADC_Sample_Time_t;

typedef struct {
	ADC_Conversion_Mode_t ConversionMode;
	uint8_t ChSelect;
	ADC_Resolution_t Resolution;
	ADC_Data_Alignment_t DataAlign;
	ADC_Sample_Time_t SampleTime;
	uint8_t TrigSource;
	uint8_t DMA_En;
	uint8_t EOC_IT_En;
	uint8_t injected_en;
}ADC_Handle_t;

/**
 * @brief
 * @param
 */
void ADC_PCLK_CTRL(ADC_TypeDef *pADCx, FunctionalState ENorDI);

/**
 * @brief
 * @param
 */
void ADC_INIT(ADC_Handle_t ADC_HANDLE);

/**
 * @brief
 * @param
 */
void ADC_DEINIT(ADC_TypeDef *pADCx);


#endif /* INC_STM32F44XX_ADC_H_ */
