/*
 * dclk_init.h
 *
 *  Created on: 10 Jan 2026
 *      Author: katog
 */

#ifndef INC_DCLK_INIT_H_
#define INC_DCLK_INIT_H_

#include "stm32f44xx_i2c.h"
#include "stm32f44xx_gpio.h"


/* I2C BUS INIT */
/**
 * @brief
 * @param
 */
void I2C_DCLK_BUS_INIT(I2C_Handle_t* I2C_HANDLE);

/* I2C PINS INIT */
/**
 * @brief
 * @param
 */
void I2C_DCLK_BUS_GPIO_INIT(GPIO_Handle_t* GPIO_I2C_HANDLE);

/* DHT22 PIN INIT */
/**
 * @brief
 * @param
 */
void DIGILOCK_DHT22_PIN_INITS(GPIO_Handle_t* DHT22_GPIO_HANDLE);

/* GPIO BUTTONS INIT */
/**
 * @brief
 * @param
 */
void DCLK_BUTTON_PINS_INIT();


#endif /* INC_DCLK_INIT_H_ */
