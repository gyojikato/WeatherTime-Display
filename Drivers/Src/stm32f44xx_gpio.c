/*
 * gpio.c
 *
 *  Created on: Apr 16, 2025
 *      Author: katog
 *
 *  Description: Implementation file for STM32F44xx GPIO driver.
 *  Provides functions to initialize, read, write, toggle GPIO pins and ports,
 *  configure EXTI interrupts, and control NVIC for GPIO interrupts.
 */

#include "stm32f44xx_gpio.h"

/********************************************
 * HELPER FUNCTIONS
 ********************************************/

/**
 * @brief Helper function to set a specific bit in a register after clearing
 *        bits defined by a mask.
 * @param REG: Pointer to the register to modify
 * @param bit_pos: Bit position to modify
 * @param mask: Value to set after clearing
 * @param clear_mask: Mask of bits to clear before setting
 */
static void SetBit(uint32_t *REG, uint16_t bit_pos, uint8_t mask,  uint8_t clear_mask)
{
	MODIFY_REG(*REG, (clear_mask << bit_pos), (mask << bit_pos));
}

/**
 * @brief Helper function to initialize EXTI (external interrupt) for a GPIO pin.
 *        Configures SYSCFG EXTICR, EMR, IMR, and trigger selection (RTSR/FTSR).
 * @param pGPIOx: Pointer to GPIO peripheral
 * @param EXTI_HANDLE: Pointer to EXTI configuration structure
 * @param PIN_NUMBER: GPIO pin number (0-15)
 * @return SUCCESS if initialization was successful, ERROR otherwise
 */
static uint8_t EXTI_INIT(GPIO_TypeDef* pGPIOx, EXTI_Config_t* EXTI_HANDLE, uint8_t PIN_NUMBER)
{
	// Enable system configuration controller clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN_Msk;

	// Select GPIO port for the EXTI line in SYSCFG_EXTICR register
	if(pGPIOx == GPIOA){
		SYSCFG->EXTICR[PIN_NUMBER / 4] &= ~(0x0F << ((PIN_NUMBER % 4) * 4));
	}
	else if(pGPIOx == GPIOB){
		SYSCFG->EXTICR[PIN_NUMBER / 4] |= (0x01 << ((PIN_NUMBER % 4) * 4));
	}
	else if(pGPIOx == GPIOC){
		SYSCFG->EXTICR[PIN_NUMBER / 4] |= (0x02 << ((PIN_NUMBER % 4) * 4));
	}
	else if(pGPIOx == GPIOD){
		SYSCFG->EXTICR[PIN_NUMBER / 4] |= (0x03 << ((PIN_NUMBER % 4) * 4));
	}
	else {
		return ERROR; // Invalid GPIO port
	}

	// Configure EXTI EMR (Event mask register)
	if(EXTI_HANDLE->EXTI_EMR_SET == DISABLE){
		EXTI->EMR &= ~(1 << PIN_NUMBER);
	} else {
		EXTI->EMR |= (1 << PIN_NUMBER);
	}

	// Configure EXTI IMR (Interrupt mask register)
	if(EXTI_HANDLE->EXTI_IMR_SET == DISABLE){
		EXTI->IMR &= ~(1 << PIN_NUMBER);
	} else {
		EXTI->IMR |= (1 << PIN_NUMBER);
	}

	// Configure EXTI trigger selection
	if(EXTI_HANDLE->EXTI_RTSR_SEL == ENABLE){
		EXTI->FTSR &= ~(1 << PIN_NUMBER);
		EXTI->RTSR |= (1 << PIN_NUMBER); // Rising edge trigger
	} else if(EXTI_HANDLE->EXTI_FTSR_SEL == ENABLE){
		EXTI->RTSR &= ~(1 << PIN_NUMBER);
		EXTI->FTSR |= (1 << PIN_NUMBER); // Falling edge trigger
	} else {
		EXTI->FTSR &= ~(1 << PIN_NUMBER);
		EXTI->RTSR &= ~(1 << PIN_NUMBER); // No trigger
	}

	return SUCCESS;
}

/********************************************
 * GPIO FUNCTION DEFINITIONS
 ********************************************/

/**
 * @brief Initialize a GPIO pin according to the user configurations.
 *        Supports input, output, analog, and alternate function modes.
 *        Configures speed, output type, pull-up/pull-down, and EXTI if enabled.
 * @param GPIO_Handle: Pointer to GPIO handle structure containing configuration
 * @return SUCCESS if initialization was successful, ERROR otherwise
 */
uint8_t GPIO_INIT(GPIO_Handle_t* GPIO_Handle)
{
	uint32_t tempreg;

	// Enable clock for GPIO peripheral
	GPIO_PCLK_CTRL(GPIO_Handle->pGPIOx, ENABLE);

	// Validate pin number
	if(GPIO_Handle->GPIO_PIN_NUMBER > 15){
		return ERROR;
	}

	// Configure pin mode
	tempreg = GPIO_Handle->pGPIOx->MODER;
	if(GPIO_Handle->GPIO_MODE == GPIO_PIN_MODE_INPUT){
		// Clear mode bits for input
		tempreg &= ~(0x03 << (GPIO_Handle->GPIO_PIN_NUMBER * 2));

		// Initialize EXTI if configured
		if(GPIO_Handle->EXTI_CFG.EXTI_IMR_SET || GPIO_Handle->EXTI_CFG.EXTI_EMR_SET){
			if(EXTI_INIT(GPIO_Handle->pGPIOx, &GPIO_Handle->EXTI_CFG, GPIO_Handle->GPIO_PIN_NUMBER) == ERROR){
				return ERROR;
			}
		}
	} else if(GPIO_Handle->GPIO_MODE <= GPIO_PIN_MODE_ANALOG){
		// Set mode bits for output, analog, or alternate function
		SetBit(&tempreg, (GPIO_Handle->GPIO_PIN_NUMBER * 2), GPIO_Handle->GPIO_MODE, 0x03);
	} else {
		return ERROR;
	}
	GPIO_Handle->pGPIOx->MODER = tempreg;

	// Configure alternate function if mode is AFIO
	if(GPIO_Handle->GPIO_MODE == GPIO_PIN_MODE_AFIO){
		tempreg = GPIO_Handle->pGPIOx->AFR[GPIO_Handle->GPIO_PIN_NUMBER / 8];
		uint8_t bit_pos = GPIO_Handle->GPIO_PIN_NUMBER % 8;

		if(GPIO_Handle->GPIO_AFIO_MODE == AFIO_MODE_0){
			tempreg &= ~(0x0F << (bit_pos * 4));
		} else if(GPIO_Handle->GPIO_AFIO_MODE <= AFIO_MODE_15){
			SetBit(&tempreg, bit_pos * 4, GPIO_Handle->GPIO_AFIO_MODE, 0x0F);
		} else {
			return ERROR;
		}
		GPIO_Handle->pGPIOx->AFR[GPIO_Handle->GPIO_PIN_NUMBER / 8] = tempreg;
	}

	// Configure output speed and type (if not input)
	if(GPIO_Handle->GPIO_MODE != GPIO_PIN_MODE_INPUT){
		// Output speed
		tempreg = GPIO_Handle->pGPIOx->OSPEEDR;
		if(GPIO_Handle->GPIO_OUTPUT_SPD == GPIO_OUT_LOW_SPD){
			tempreg &= ~(3 << (GPIO_Handle->GPIO_PIN_NUMBER * 2));
		} else if(GPIO_Handle->GPIO_OUTPUT_SPD <= GPIO_OUT_HIGH_SPD){
			SetBit(&tempreg, (GPIO_Handle->GPIO_PIN_NUMBER * 2), GPIO_Handle->GPIO_OUTPUT_SPD, 0x03);
		} else {
			return ERROR;
		}
		GPIO_Handle->pGPIOx->OSPEEDR = tempreg;

		// Output type
		tempreg = GPIO_Handle->pGPIOx->OTYPER;
		if(GPIO_Handle->GPIO_OUTPUT_TYPE == GPIO_OUT_PP){
			tempreg &= ~(1 << GPIO_Handle->GPIO_PIN_NUMBER); // Push-pull
		} else if(GPIO_Handle->GPIO_OUTPUT_TYPE == GPIO_OUT_OD){
			SetBit(&tempreg, GPIO_Handle->GPIO_PIN_NUMBER, 0x01, 0x01); // Open-drain
		} else {
			return ERROR;
		}
		GPIO_Handle->pGPIOx->OTYPER = tempreg;
	}

	// Configure pull-up/pull-down
	tempreg = GPIO_Handle->pGPIOx->PUPDR;
	if(GPIO_Handle->GPIO_PUPPD == GPIO_NO_PUPD){
		tempreg &= ~(0x03 << (GPIO_Handle->GPIO_PIN_NUMBER * 2));
	} else if(GPIO_Handle->GPIO_PUPPD <= GPIO_PULL_DOWN){
		SetBit(&tempreg, GPIO_Handle->GPIO_PIN_NUMBER * 2, GPIO_Handle->GPIO_PUPPD, 0x03);
	} else {
		return ERROR;
	}
	GPIO_Handle->pGPIOx->PUPDR = tempreg;

	return SUCCESS;
}

/**
 * @brief De-initialize a GPIO peripheral.
 *        Resets registers to default reset state.
 * @param GPIO_Handle: Pointer to GPIO handle
 */
void GPIO_DeINIT(GPIO_Handle_t* GPIO_Handle)
{
	// Not implemented yet, could reset MODER, OTYPER, OSPEEDR, PUPDR, AFR
}

/**
 * @brief Enable or disable the peripheral clock for a GPIO port.
 * @param pGPIOx: Pointer to GPIO port (GPIOA-GPIOD)
 * @param ENorDI: ENABLE or DISABLE clock
 */
void GPIO_PCLK_CTRL(GPIO_TypeDef* pGPIOx, FunctionalState ENorDI)
{
	if(ENorDI == ENABLE){
		if(pGPIOx == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN_Msk;
		else if(pGPIOx == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN_Msk;
		else if(pGPIOx == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN_Msk;
		else if(pGPIOx == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN_Msk;
	}
	else {
		if(pGPIOx == GPIOA) RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN_Msk;
		else if(pGPIOx == GPIOB) RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN_Msk;
		else if(pGPIOx == GPIOC) RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN_Msk;
		else if(pGPIOx == GPIOD) RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN_Msk;
	}
}

/**
 * @brief Read input value from a specific GPIO pin
 * @param pGPIOx: Pointer to GPIO port
 * @param PIN_NUMBER: Pin number (0-15)
 * @return 0 if low, 1 if high
 */
uint8_t GPIO_READ_INPUT_PIN(GPIO_TypeDef* pGPIOx, uint8_t PIN_NUMBER)
{
	return (pGPIOx->IDR >> PIN_NUMBER) & 0x01;
}

/**
 * @brief Read input value from an entire GPIO port
 * @param pGPIOx: Pointer to GPIO port
 * @return 16-bit value of all pins
 */
uint16_t GPIO_READ_INPUT_PORT(GPIO_TypeDef* pGPIOx)
{
	return pGPIOx->IDR & 0xFFFF;
}

/**
 * @brief Write output value to a specific GPIO pin
 * @param pGPIOx: Pointer to GPIO port
 * @param PIN_NUMBER: Pin number (0-15)
 * @param value: SET or RESET
 */
void GPIO_WRITE_OUTPUT_PIN(GPIO_TypeDef* pGPIOx, uint8_t PIN_NUMBER, FlagStatus value)
{
	if(PIN_NUMBER > 15) return;

	if(value == RESET) pGPIOx->ODR &= ~(1 << PIN_NUMBER);
	else if(value == SET) pGPIOx->ODR |= (1 << PIN_NUMBER);
}

/**
 * @brief Write a 16-bit value to the entire GPIO port
 * @param pGPIOx: Pointer to GPIO port
 * @param value: 16-bit value to write to ODR
 */
void GPIO_WRITE_OUTPUT_PORT(GPIO_TypeDef* pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value & 0xFFFF;
}

/**
 * @brief Toggle a GPIO pin state (high <-> low)
 * @param pGPIOx: Pointer to GPIO port
 * @param PIN_NUMBER: Pin number (0-15)
 */
void GPIO_TOGGLE_PIN(GPIO_TypeDef* pGPIOx, uint8_t PIN_NUMBER)
{
	if(PIN_NUMBER > 15) return;
	pGPIOx->ODR ^= (1 << PIN_NUMBER);
}

/**
 * @brief Enable or disable NVIC interrupt for a specific IRQ
 * @param IRQ_NUMBER: IRQ number
 * @param ENorDI: ENABLE or DISABLE interrupt
 */
void GPIO_IRQ_CFG(uint8_t IRQ_NUMBER, FunctionalState ENorDI)
{
	if(ENorDI == ENABLE) NVIC_EnableIRQ(IRQ_NUMBER);
	else if(ENorDI == DISABLE) NVIC_DisableIRQ(IRQ_NUMBER);
}

/**
 * @brief Handle a pending EXTI interrupt for a specific pin
 *        Clears the pending bit to allow future interrupts.
 * @param PIN_NUMBER: Pin number (0-15)
 */
void GPIO_IRQ_HANDLING(uint8_t PIN_NUMBER)
{
	if(PIN_NUMBER > 15) return;

	if(EXTI->PR & (1 << PIN_NUMBER)){
		EXTI->PR |= (1 << PIN_NUMBER); // Clear pending bit
	}
}
