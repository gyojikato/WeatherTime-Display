/*
 * ds1307_wrappers.c
 *
 *  Created on: 10 Jan 2026
 *      Author: katog
 */


/**
 * @file ds1307_wrappers.c
 * @brief FreeRTOS-aware I2C wrapper functions for DS1307 RTC communication.
 *
 * These functions wrap the low-level non-blocking I2C driver APIs and provide
 * timeout-protected access suitable for use inside FreeRTOS tasks.
 */

#include "ds1307_wrappers.h"

/**
 * @brief I2C handle used for DS1307 communication.
 *
 * This handle must be initialized before calling any DS1307 wrapper functions.
 * Access to the I2C peripheral is expected to be protected externally using
 * a semaphore or mutex.
 */
I2C_Handle_t  I2C_DCLK_BUS_h;

/* DS1307 WRAPPER FUNCTIONS */

/**
 * @brief Write data to DS1307 over I2C with timeout protection.
 *
 * This function repeatedly attempts a non-blocking I2C transmit operation
 * until the bus becomes available or a timeout occurs.
 *
 * @param Slave_Addr: 7-bit I2C slave address of the DS1307
 * @param TxBuffer:   Pointer to transmit buffer
 * @param len:        Number of bytes to transmit
 *
 * @return 1 on success, 0 on timeout or failure
 */
uint8_t DS1307_I2C_WRITE(uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len)
{
	TimeOut_t xTimeOut;
	TickType_t ticks_to_wait = pdMS_TO_TICKS(100);
	vTaskSetTimeOutState(&xTimeOut);
	while(I2C_MSTR_SEND_DATA_IT(&I2C_DCLK_BUS_h, Slave_Addr, TxBuffer, len, DISABLE) == I2C_BUSY){
		if(xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE){
			return 0;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	return 1;
}

/**
 * @brief Read data from DS1307 over I2C with timeout protection.
 *
 * Performs a register address write followed by a non-blocking I2C read.
 * Both phases are protected with a FreeRTOS timeout mechanism.
 *
 * @param Slave_Addr 7-bit I2C slave address of the DS1307
 * @param reg_addr   Register address to read from
 * @param RxBuffer   Pointer to receive buffer
 * @param len        Number of bytes to read
 *
 * @return 1 on success, 0 on timeout or failure
 */
uint8_t DS1307_I2C_READ(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t* RxBuffer, uint32_t len)
{
	TimeOut_t xTimeOut;
	TickType_t ticks_to_wait = pdMS_TO_TICKS(300);
	vTaskSetTimeOutState(&xTimeOut);
	while(I2C_MSTR_SEND_DATA_IT(&I2C_DCLK_BUS_h, Slave_Addr, &reg_addr, 1, ENABLE) == I2C_BUSY){
		if(xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE){

			return 0;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	vTaskSetTimeOutState(&xTimeOut);
	while(I2C_MSTR_RECEIVE_DATA_IT(&I2C_DCLK_BUS_h, Slave_Addr, RxBuffer, len, DISABLE) == I2C_BUSY){
		if(xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE){

			return 0;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	return 1;
}
