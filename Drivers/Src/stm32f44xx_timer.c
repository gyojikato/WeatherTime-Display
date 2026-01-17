/*
 * stm32f44xx_timer.c
 *
 *  Created on: Apr 21, 2025
 *      Author: katog
 *
 *  @brief Timer driver source file for STM32F44xx
 */

#include "stm32f44xx_timer.h"

/* ============================================================
 *                      MACROS
 * ============================================================ */

/**
 * @brief Timer clock frequency after prescaler (1 MHz)
 */
#define TIM_CLK_FREQ        (1000000)

/**
 * @brief Timeout value for timer initialization
 */
#define TIMER_TIMEOUT       (16000)   /* ~1 ms timeout at 16 MHz system clock */

/* ============================================================
 *                      DELAY DRIVER (TIM6)
 * ============================================================ */

/*
 * TIM6 is reserved exclusively for delay functionality.
 *
 * Configuration:
 *  - Prescaler: 16 - 1  -> 1 MHz timer clock
 *  - Auto-reload: Max (0xFFFF)
 *  - Counter tick: 1 us
 *  - Control register: default configuration
 */

/**
 * @brief Initializes TIM6 as a delay source
 * @retval 1 if initialization successful, 0 on timeout
 *
 * @note TIM6 must not be used for any purpose other than delays.
 */
uint8_t DELAY_INIT(void)
{
	/* Enable TIM6 peripheral clock */
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN_Msk;

	/* Reset control register to default */
	TIM6->CR1 = 0;

	/* Set auto-reload register to maximum */
	TIM6->ARR = 0xFFFF;

	/* APB1 clock = 16 MHz, prescaler = 16 - 1 → 1 MHz timer clock */
	TIM6->PSC = 0x1F - 1;

	/* Force update event to load prescaler and ARR */
	TIM6->EGR |= TIM_EGR_UG_Msk;

	/* Wait until update event flag is set */
	uint32_t temp_cnt = 0;
	while(!(TIM6->SR & TIM_SR_UIF_Msk))
	{
		if(temp_cnt++ > TIMER_TIMEOUT)
			return 0;  /* Prevent lock-up */
	}

	/* Enable timer counter */
	TIM6->CR1 |= TIM_CR1_CEN_Msk;

	return 1;
}

/**
 * @brief Generates blocking delay in microseconds
 * @param delay Delay duration in microseconds
 */
void DELAY_us(uint32_t delay)
{
	TIM6->CNT = 0;
	while(TIM6->CNT < delay);
}

/**
 * @brief Generates blocking delay in milliseconds
 * @param delay Delay duration in milliseconds
 */
void DELAY_ms(uint32_t delay)
{
	while(delay--)
	{
		DELAY_us(1000);
	}
}

/**
 * @brief Returns current timer tick value
 * @retval TIM6 counter value (1 tick = 1 us)
 */
uint32_t DELAY_TICK(void)
{
	return TIM6->CNT;
}

/* ============================================================
 *                      PWM DRIVER
 * ============================================================ */

/*
 * PWM functionality is also implemented using general-purpose timers.
 *
 * Notes:
 *  - Recommended timers: TIM2–TIM5 or TIM9–TIM14
 *  - All timers are configured to 1 MHz clock for easier calculations
 *  - Supports dynamic duty cycle and frequency changes
 */

/**
 * @brief Initializes PWM output
 * @param TIM_HANDLE Pointer to PWM handle structure
 *
 * @note Timer clock is configured to 1 MHz.
 * @note ARR and CCR are computed based on desired frequency and duty cycle.
 */
void PWM_INIT(PWM_Handle_t* TIM_HANDLE)
{
	uint32_t ARR_VAL;
	uint32_t CCR_VAL;

	/* Enable TIM2 peripheral clock (default PWM timer) */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN_Msk;

	/* Set timer clock to 1 MHz (16 MHz / 16) */
	TIM_HANDLE->pTIMx->PSC = 15;

	/* Reset control register */
	TIM_HANDLE->pTIMx->CR1 = 0;

	/* Compute auto-reload value for desired PWM frequency */
	ARR_VAL = TIM_CLK_FREQ / TIM_HANDLE->PWM_FREQUENCY;
	TIM_HANDLE->pTIMx->ARR = ARR_VAL - 1;

	/* Compute compare value for duty cycle (0–100%) */
	CCR_VAL = (ARR_VAL * TIM_HANDLE->PWM_DUTY_CYCLE) / 100;
	TIM_HANDLE->pTIMx->CCR[TIM_HANDLE->PWM_CHANNEL] = CCR_VAL;

	/* Configure PWM mode 1 (active high) */
	TIM_HANDLE->pTIMx->CCMR[TIM_HANDLE->PWM_CHANNEL / 2] |=
		(0x06 << (4 + (8 * (TIM_HANDLE->PWM_CHANNEL % 2))));

	/* Enable preload for CCR register */
	TIM_HANDLE->pTIMx->CCMR[TIM_HANDLE->PWM_CHANNEL / 2] |=
		(1 << (3 + (8 * (TIM_HANDLE->PWM_CHANNEL % 2))));

	/* Configure output compare polarity */
	TIM_HANDLE->pTIMx->CCER |=
		(TIM_HANDLE->PWM_OC_ACTIVE_STATE << (1 + (4 * TIM_HANDLE->PWM_CHANNEL)));

	/* Generate update event to load registers */
	TIM2->EGR |= TIM_EGR_UG_Msk;

	/* Wait until update flag is cleared */
	while(TIM2->SR & TIM_SR_UIF_Msk);

	/* Enable timer */
	TIM2->CR1 |= TIM_CR1_CEN_Msk;
}

/**
 * @brief Enables or disables a PWM channel
 * @param pTIMx Pointer to timer peripheral
 * @param CH PWM channel selection
 * @param ENorDI ENABLE or DISABLE
 */
void PWM_CH_CTRL(TIM_TypeDef* pTIMx, PWM_CH_SEL_t CH, FunctionalState ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pTIMx->CCER |= (1 << (4 * CH));
	}
	else
	{
		pTIMx->CCER &= ~(1 << (4 * CH));
	}
}

/**
 * @brief Dynamically updates PWM duty cycle and frequency
 * @param pTIMx Pointer to timer peripheral
 * @param CH PWM channel selection
 * @param DUTY_CYCLE Duty cycle percentage (0–100)
 * @param FREQUENCY PWM frequency in Hz
 */
void PWM_SET_DC_FQ(TIM_TypeDef* pTIMx,
                   PWM_CH_SEL_t CH,
                   uint8_t DUTY_CYCLE,
                   uint32_t FREQUENCY)
{
	uint32_t ARR_VAL;
	uint32_t CCR_VAL;

	/* Update frequency */
	ARR_VAL = TIM_CLK_FREQ / FREQUENCY;
	pTIMx->ARR = ARR_VAL - 1;

	/* Update duty cycle */
	CCR_VAL = (ARR_VAL * DUTY_CYCLE) / 100;
	pTIMx->CCR[CH] = CCR_VAL;

	/* Reset counter to apply changes immediately */
	pTIMx->CNT = 0;
}
