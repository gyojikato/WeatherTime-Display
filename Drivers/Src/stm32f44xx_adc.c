/*
 * stm32f44xx_adc.c
 *
 *  Created on: 7 Sep 2025
 *      Author: katog
 */


#include "stm32f44xx_adc.h"


/**
 * @brief
 * @param
 */
void ADC_PCLK_CTRL(ADC_TypeDef *pADCx, FunctionalState ENorDI)
{
	if(ENorDI){
		if(pADCx == ADC1){

		}
		else if(pADCx == ADC2){

		}
		else if(pADCx == ADC3){

		}
	}
	else {
		if(pADCx == ADC1){

		}
		else if(pADCx == ADC2){

		}
		else if(pADCx == ADC3){

		}
	}
}

/**
 * @brief
 * @param
 */
void ADC_INIT(ADC_Handle_t ADC_HANDLE);

/**
 * @brief
 * @param
 */
void ADC_DEINIT(ADC_TypeDef *pADCx){
	if(pADCx == ADC1){

	}
	else if(pADCx == ADC2){

	}
	else if(pADCx == ADC3){

	}
}

