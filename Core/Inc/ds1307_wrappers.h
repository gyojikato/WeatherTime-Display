/*
 * ds1307_wrappers.h
 *
 *  Created on: 10 Jan 2026
 *      Author: katog
 */

#ifndef INC_DS1307_WRAPPERS_H_
#define INC_DS1307_WRAPPERS_H_

#include "stdint.h"
#include "stm32f44xx_i2c.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "dclk_tasks.h"

/**
 * @brief I2C handle used for DS1307 communication.
 *
 * This handle must be initialized before calling any DS1307 wrapper functions.
 * Access to the I2C peripheral is expected to be protected externally using
 * a semaphore or mutex.
 */
extern I2C_Handle_t  I2C_DCLK_BUS_h;
/* DS1307 WRAPPER FUNCTIONS */
/**
 * @brief
 * @param
 */
uint8_t DS1307_I2C_WRITE(uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len);

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
uint8_t DS1307_I2C_READ(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t* RxBuffer, uint32_t len);


#endif /* INC_DS1307_WRAPPERS_H_ */
