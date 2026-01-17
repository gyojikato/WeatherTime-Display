/*
 * dht22_wrappers.c
 *
 *  Created on: 10 Jan 2026
 *      Author: katog
 */

#include "dht22_wrappers.h"

/**
 * @brief GPIO handle for DHT22 data pin (PC9).
 *
 * This handle is configured during system initialization and reused
 * by the DHT22 wrapper functions.
 */
GPIO_Handle_t DHT22_PIN_h;

/* DHT22 WRAPPER FUNCTIONS */

/**
 * @brief Drive the DHT22 data pin output level.
 *
 * @param value Output level (0 = LOW, non-zero = HIGH)
 */
void DHT22_WRITE_PIN(uint8_t value)
{
	GPIO_WRITE_OUTPUT_PIN(GPIOC, GPIO_PIN_NUMBER_9, value);
}

/**
 * @brief Read the current logic level of the DHT22 data pin.
 *
 * @return GPIO input level (0 = LOW, 1 = HIGH)
 */
uint8_t DHT22_READ_PIN(void)
{
	return GPIO_READ_INPUT_PIN(GPIOC, GPIO_PIN_NUMBER_9);
}

/**
 * @brief Configure the DHT22 data pin direction.
 *
 * Switches the GPIO pin between input and output modes as required
 * by the DHT22 communication protocol.
 *
 * @param PIN_MODE Desired DHT22 pin mode (INPUT or OUTPUT)
 */
void DHT22_PIN_MODE(DHT22_PIN_MODE_t PIN_MODE)
{
	if(PIN_MODE == DHT22_PIN_MODE_INPUT){
		DHT22_PIN_h.GPIO_MODE = GPIO_PIN_MODE_INPUT;
	}
	else {
		DHT22_PIN_h.GPIO_MODE = GPIO_PIN_MODE_OUTPUT;
	}

	GPIO_INIT(&DHT22_PIN_h);
}

/**
 * @brief Busy-wait delay in microseconds.
 *
 * Used to satisfy the strict timing requirements of the DHT22 protocol.
 *
 * @param delay Delay duration in microseconds
 */
void DHT22_us_DELAY(uint32_t delay)
{
	DELAY_us(delay);
}

/**
 * @brief Get the current system tick value.
 *
 * Used by the DHT22 driver for timing and timeout measurements.
 *
 * @return Current tick count
 */
uint32_t DHTT22_CURR_TICK(void)
{
	return DELAY_TICK();
}
