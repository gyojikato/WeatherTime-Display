/*
 * dht22_wrappers.h
 *
 *  Created on: 10 Jan 2026
 *      Author: katog
 */

#ifndef INC_DHT22_WRAPPERS_H_
#define INC_DHT22_WRAPPERS_H_

#include <stdint.h>
#include "stm32f44xx_gpio.h"
#include "stm32f44xx_timer.h"
#include "DHT22.h"

/**
 * @brief GPIO handle used for DHT22 data pin.
 *
 * This handle must be initialized before calling any DHT22 driver functions.
 */
extern GPIO_Handle_t DHT22_PIN_h;

/* DHT22 WRAPPER FUNCTIONS */

/**
 * @brief Drive the DHT22 data pin output level.
 *
 * @param value GPIO output value (0 = LOW, non-zero = HIGH)
 */
void DHT22_WRITE_PIN(uint8_t value);

/**
 * @brief Read the current logic level of the DHT22 data pin.
 *
 * @return GPIO input state (0 = LOW, 1 = HIGH)
 */
uint8_t DHT22_READ_PIN(void);

/**
 * @brief Configure the DHT22 data pin mode.
 *
 * @param PIN_MODE Desired pin mode (INPUT or OUTPUT)
 */
void DHT22_PIN_MODE(DHT22_PIN_MODE_t PIN_MODE);

/**
 * @brief Busy-wait delay in microseconds for DHT22 timing requirements.
 *
 * @param delay Delay duration in microseconds
 */
void DHT22_us_DELAY(uint32_t delay);

/**
 * @brief Get the current system tick value.
 *
 * Used by the DHT22 driver for timeout and timing measurements.
 *
 * @return Current tick count
 */
uint32_t DHTT22_CURR_TICK(void);

#endif /* INC_DHT22_WRAPPERS_H_ */
