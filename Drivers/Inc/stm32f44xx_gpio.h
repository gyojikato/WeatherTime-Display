/*
 * gpio.h
 *
 *  Created on: Apr 16, 2025
 *      Author: katog
 *
 *  Description: Header file for STM32F44xx GPIO driver.
 *  Provides macros, data structures, enumerations, and function
 *  prototypes for GPIO peripheral configuration and usage.
 */

#ifndef INC_STM32F44XX_GPIO_H_
#define INC_STM32F44XX_GPIO_H_

#include "stm32f4xx.h"

/********************************************
 * GPIO PIN DEFINITIONS
 ********************************************/
// GPIO pin numbers (0-15)
#define GPIO_PIN_NUMBER_0        (0)
#define GPIO_PIN_NUMBER_1        (1)
#define GPIO_PIN_NUMBER_2        (2)
#define GPIO_PIN_NUMBER_3        (3)
#define GPIO_PIN_NUMBER_4        (4)
#define GPIO_PIN_NUMBER_5        (5)
#define GPIO_PIN_NUMBER_6        (6)
#define GPIO_PIN_NUMBER_7        (7)
#define GPIO_PIN_NUMBER_8        (8)
#define GPIO_PIN_NUMBER_9        (9)
#define GPIO_PIN_NUMBER_10       (10)
#define GPIO_PIN_NUMBER_11       (11)
#define GPIO_PIN_NUMBER_12       (12)
#define GPIO_PIN_NUMBER_13       (13)
#define GPIO_PIN_NUMBER_14       (14)
#define GPIO_PIN_NUMBER_15       (15)

/********************************************
 * ALTERNATE FUNCTION DEFINITIONS (AFIO)
 ********************************************/
// Alternate function mode numbers for GPIO_AF registers
#define AFIO_MODE_0             (0)   // System function
#define AFIO_MODE_1             (1)   // TIM1/TIM2
#define AFIO_MODE_4             (4)   // I2C1/I2C2/I2C3/I2C4
#define AFIO_MODE_5             (5)   // SPI1/SPI2/SPI3/SPI4
#define AFIO_MODE_7             (7)   // SPI2/SPI3, USART1/2/3, USART4/5
#define AFIO_MODE_8             (8)   // USART6, USART4/5
#define AFIO_MODE_15            (15)  // System function

/********************************************
 * ENUMERATIONS
 ********************************************/

/**
 * @brief GPIO pin modes
 */
typedef enum {
	GPIO_PIN_MODE_INPUT,    // Input mode
	GPIO_PIN_MODE_OUTPUT,   // General purpose output mode
	GPIO_PIN_MODE_AFIO,     // Alternate function mode
	GPIO_PIN_MODE_ANALOG    // Analog mode
} GPIO_PIN_MODE_t;

/**
 * @brief GPIO output speeds
 */
typedef enum {
	GPIO_OUT_LOW_SPD,   // Low speed
	GPIO_OUT_MED_SPD,   // Medium speed
	GPIO_OUT_FST_SPD,   // Fast speed
	GPIO_OUT_HIGH_SPD   // High speed
} GPIO_OUT_SPD_t;

/**
 * @brief GPIO output types
 */
typedef enum {
	GPIO_OUT_PP,    // Push-Pull
	GPIO_OUT_OD     // Open-Drain
} GPIO_OTYPE_t;

/**
 * @brief GPIO pull-up/pull-down configuration
 */
typedef enum {
	GPIO_NO_PUPD,   // No pull-up or pull-down
	GPIO_PULL_UP,   // Pull-up enabled
	GPIO_PULL_DOWN  // Pull-down enabled
} GPIO_PUPPD_t;

/********************************************
 * EXTERNAL INTERRUPT CONFIGURATION
 ********************************************/

/**
 * @brief Structure to configure EXTI (External Interrupt)
 */
typedef struct {
	FunctionalState EXTI_IMR_SET;   // Interrupt mask (IMR)
	FunctionalState EXTI_EMR_SET;   // Event mask (EMR)
	FunctionalState EXTI_FTSR_SEL;  // Falling edge trigger selection
	FunctionalState EXTI_RTSR_SEL;  // Rising edge trigger selection
} EXTI_Config_t;

/********************************************
 * GPIO HANDLE STRUCTURE
 ********************************************/

/**
 * @brief GPIO handle structure for user configuration
 */
typedef struct {
	GPIO_PIN_MODE_t GPIO_MODE;        // Pin mode (input/output/AFIO/analog)
	GPIO_OUT_SPD_t GPIO_OUTPUT_SPD;   // Output speed
	uint8_t GPIO_PIN_NUMBER;          // Pin number (0-15)
	GPIO_OTYPE_t GPIO_OUTPUT_TYPE;    // Output type (push-pull/open-drain)
	GPIO_PUPPD_t GPIO_PUPPD;          // Pull-up/pull-down
	uint8_t GPIO_AFIO_MODE;           // Alternate function selection (0-15)
	EXTI_Config_t EXTI_CFG;           // EXTI configuration (if input with interrupt)
	GPIO_TypeDef* pGPIOx;             // Pointer to GPIO peripheral (GPIOA-GPIOD)
} GPIO_Handle_t;

/********************************************
 * FUNCTION PROTOTYPES
 ********************************************/

/**
 * @brief Initialize GPIO pin based on user configuration
 * @param GPIO_Handle: Pointer to GPIO handle structure
 * @return SUCCESS or ERROR
 */
uint8_t GPIO_INIT(GPIO_Handle_t* GPIO_Handle);

/**
 * @brief De-initialize GPIO peripheral
 * @param GPIO_Handle: Pointer to GPIO handle structure
 */
void GPIO_DeINIT(GPIO_Handle_t* GPIO_Handle);

/**
 * @brief Enable or disable GPIO peripheral clock
 * @param pGPIOx: Pointer to GPIO port
 * @param ENorDI: ENABLE or DISABLE
 */
void GPIO_PCLK_CTRL(GPIO_TypeDef* pGPIOx, FunctionalState ENorDI);

/**
 * @brief Read input value from a specific GPIO pin
 * @param pGPIOx: Pointer to GPIO port
 * @param PIN_NUMBER: Pin number (0-15)
 * @return 0 if low, 1 if high
 */
uint8_t GPIO_READ_INPUT_PIN(GPIO_TypeDef* pGPIOx, uint8_t PIN_NUMBER);

/**
 * @brief Read input value from an entire GPIO port
 * @param pGPIOx: Pointer to GPIO port
 * @return 16-bit port value
 */
uint16_t GPIO_READ_INPUT_PORT(GPIO_TypeDef* pGPIOx);

/**
 * @brief Write output value to a specific GPIO pin
 * @param pGPIOx: Pointer to GPIO port
 * @param PIN_NUMBER: Pin number (0-15)
 * @param value: SET or RESET
 */
void GPIO_WRITE_OUTPUT_PIN(GPIO_TypeDef* pGPIOx, uint8_t PIN_NUMBER, FlagStatus value);

/**
 * @brief Write a 16-bit value to the entire GPIO port
 * @param pGPIOx: Pointer to GPIO port
 * @param value: 16-bit value to write
 */
void GPIO_WRITE_OUTPUT_PORT(GPIO_TypeDef* pGPIOx, uint16_t value);

/**
 * @brief Toggle the state of a specific GPIO pin
 * @param pGPIOx: Pointer to GPIO port
 * @param PIN_NUMBER: Pin number (0-15)
 */
void GPIO_TOGGLE_PIN(GPIO_TypeDef* pGPIOx, uint8_t PIN_NUMBER);

/**
 * @brief Enable or disable NVIC interrupt for a given IRQ
 * @param IRQ_NUMBER: NVIC IRQ number
 * @param ENorDI: ENABLE or DISABLE
 */
void GPIO_IRQ_CFG(uint8_t IRQ_NUMBER, FunctionalState ENorDI);

/**
 * @brief Handle EXTI interrupt for a specific pin
 *        Clears the pending interrupt bit
 * @param PIN_NUMBER: Pin number (0-15)
 */
void GPIO_IRQ_HANDLING(uint8_t PIN_NUMBER);

#endif /* INC_STM32F44XX_GPIO_H_ */
