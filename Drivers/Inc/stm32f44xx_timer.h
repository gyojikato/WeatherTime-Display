/*
 * stm32f44xx_timer.h
 *
 *  Created on: Apr 21, 2025
 *      Author: katog
 *
 *  @brief Timer driver header file for STM32F44xx
 */

#ifndef INC_STM32F44XX_TIMER_H_
#define INC_STM32F44XX_TIMER_H_

#include "stm32f4xx.h"

/* ============================================================
 *                      ENUMERATIONS
 * ============================================================ */

/**
 * @brief PWM channel selection
 */
typedef enum
{
	PWM_CH_SEL_CH1,    /*!< PWM Channel 1 */
	PWM_CH_SEL_CH2,    /*!< PWM Channel 2 */
	PWM_CH_SEL_CH3,    /*!< PWM Channel 3 */
	PWM_CH_SEL_CH4     /*!< PWM Channel 4 */

} PWM_CH_SEL_t;

/**
 * @brief PWM output compare mode
 */
typedef enum
{
	PWM_MODE_1 = 0x06, /*!< PWM mode 1 (active until compare match) */
	PWM_MODE_2         /*!< PWM mode 2 (inactive until compare match) */

} PWM_MODE_t;

/**
 * @brief PWM output compare polarity
 */
typedef enum
{
	PWM_OC_ACTIVE_HIGH, /*!< Active high output polarity */
	PWM_OC_ACTIVE_LOW   /*!< Active low output polarity */

} PWM_OC_ACTIVE_STATE_t;

/* ============================================================
 *                      HANDLE STRUCTURE
 * ============================================================ */

/**
 * @brief PWM handle structure
 *
 * Holds PWM configuration parameters and timer reference.
 */
typedef struct
{
	uint32_t               PWM_FREQUENCY;        /*!< PWM frequency in Hz */
	uint8_t                PWM_DUTY_CYCLE;       /*!< Duty cycle percentage (0–100) */
	PWM_CH_SEL_t           PWM_CHANNEL;           /*!< PWM channel selection */
	PWM_MODE_t             PWM_MODE;              /*!< PWM mode selection */
	PWM_OC_ACTIVE_STATE_t  PWM_OC_ACTIVE_STATE;   /*!< Output polarity */
	TIM_TypeDef*           pTIMx;                  /*!< Timer peripheral base address */

} PWM_Handle_t;

/* ============================================================
 *                      DELAY API
 * ============================================================ */

/**
 * @brief Initializes TIM6 as delay timer
 * @retval 1 if initialization successful, 0 otherwise
 */
uint8_t DELAY_INIT(void);

/**
 * @brief Generates blocking delay in microseconds
 * @param delay Delay duration in microseconds
 */
void DELAY_us(uint32_t delay);

/**
 * @brief Generates blocking delay in milliseconds
 * @param delay Delay duration in milliseconds
 */
void DELAY_ms(uint32_t delay);

/**
 * @brief Returns current delay timer tick
 * @retval Timer tick value (1 tick = 1 us)
 */
uint32_t DELAY_TICK(void);

/* ============================================================
 *                      PWM API
 * ============================================================ */

/**
 * @brief Initializes PWM output
 * @param TIM_HANDLE Pointer to PWM handle structure
 */
void PWM_INIT(PWM_Handle_t* TIM_HANDLE);

/**
 * @brief Enables or disables a PWM channel
 * @param pTIMx Pointer to timer peripheral
 * @param CH PWM channel selection
 * @param ENorDI ENABLE or DISABLE
 */
void PWM_CH_CTRL(TIM_TypeDef* pTIMx, PWM_CH_SEL_t CH, FunctionalState ENorDI);

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
                   uint32_t FREQUENCY);

#endif /* INC_STM32F44XX_TIMER_H_ */
