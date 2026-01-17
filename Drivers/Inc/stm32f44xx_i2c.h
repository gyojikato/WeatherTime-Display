/*
 * stm32f44xx_i2c.h
 *
 *  Created on: May 3, 2025
 *      Author: katog
 *
 *  Description: Header file for STM32F44xx I2C driver. Defines types,
 *  handle structure, function prototypes, and callback events for
 *  both polling and interrupt-based communication.
 */

#ifndef INC_STM32F44XX_I2C_H_
#define INC_STM32F44XX_I2C_H_

#include "stm32f4xx.h"         // STM32F4xx base addresses and registers
#include "stm32f44xx_rcc.h"    // RCC peripheral clock control
#include "stm32f44xx_timer.h"  // Timer (optional for timing/delays)
#include <stdlib.h>

/**
 * @brief Enum for possible I2C application callback events.
 * These are used to notify the user application when an event occurs.
 */
typedef enum {
	I2C_APPEV_CLLBCK_TX_CMPLT,     // Transmission complete (Master or Slave)
	I2C_APPEV_CLLBCK_RX_CMPLT,     // Reception complete (Master or Slave)
	I2C_APPEV_CLLBCK_BERR_OCCUR,   // Bus error occurred
	I2C_APPEV_CLLBCK_ARLO_OCCUR,   // Arbitration lost
	I2C_APPEV_CLLBCK_AF_OCCUR,     // Acknowledge failure
	I2C_APPEV_CLLBCK_OVR_OCCUR,    // Overrun/Underrun error
	I2C_APPEV_CLLBCK_PECERR_OCCUR, // PEC error (used in SMBus)
	I2C_APPEV_CLLBCK_TIMEOUT_OCCUR // Timeout error
} I2C_APPEV_CLLBCK_t;

/**
 * @brief Enum for possible I2C peripheral states.
 * Used to track the status of interrupt-based communication.
 */
typedef enum {
	I2C_STATUS_READY,    // Peripheral is ready for a new operation
	I2C_STATUS_TX_BUSY,  // Transmission in progress
	I2C_STATUS_RX_BUSY,  // Reception in progress
	I2C_BUSY,            // Peripheral busy
	I2C_OK               // Operation successful
} I2C_STATUS_t;

/**
 * @brief Enum for I2C polling communication return status.
 */
typedef enum {
	I2C_ERROR_TIMEOUT,  // Timeout occurred
	I2C_SUCCESS         // Operation successful
} I2C_ERROR_STATUS_t;

/**
 * @brief Enum for supported I2C bus speeds.
 */
typedef enum {
	I2C_SPEED_SM = 100000, // Standard mode: 100 kHz
	I2C_SPEED_FM = 400000  // Fast mode: 400 kHz
} I2C_SPEED_t;

/**
 * @brief Enum for Fast Mode duty cycle configuration.
 */
typedef enum {
	I2C_FM_DUTY_MODE_2,      // Duty cycle = Tlow/Thigh = 2
	I2C_FM_DUTY_MODE_16_9    // Duty cycle = 16/9
} I2C_FM_DUTY_MODE;

/**
 * @brief I2C handle structure.
 * Contains configuration parameters, buffers, lengths, and status for both
 * polling and interrupt-based communication.
 */
typedef struct {
	I2C_SPEED_t I2C_SPD;         // I2C speed (SM or FM)
	I2C_TypeDef* pI2Cx;          // Base address of I2C peripheral
	I2C_FM_DUTY_MODE I2C_FM_DUTY_MODE; // Fast mode duty cycle
	uint8_t DEVICE_ADDR;          // Own device address (for slave mode)
	uint8_t* TxBuffer;            // Pointer to transmission buffer
	uint8_t* RxBuffer;            // Pointer to reception buffer
	uint32_t TxSize;              // Total size of transmission buffer
	uint32_t RxSize;              // Total size of reception buffer
	uint32_t TxLen;               // Remaining bytes to transmit
	uint32_t RxLen;               // Remaining bytes to receive
	uint8_t SlaveAddress;         // Slave address (for master mode)
	I2C_STATUS_t I2C_STATUS;      // Current status of I2C
	uint8_t I2C_RPT_STRT;         // Repeated start flag (ENABLE/DISABLE)
} I2C_Handle_t;

/********************************************
 * Function Prototypes
 ********************************************/

/**
 * @brief Enables or disables the peripheral clock for a specific I2C peripheral.
 * @param pI2Cx: Pointer to I2C peripheral (I2C1/I2C2/I2C3).
 * @param ENorDI: ENABLE or DISABLE the clock.
 */
void I2C_PCLK_CTRL(I2C_TypeDef* pI2Cx, uint8_t ENorDI);

/**
 * @brief Initializes the I2C peripheral according to the configuration in I2C_Handle_t.
 * @param I2C_HANDLE: Pointer to I2C handle with configuration settings.
 */
void I2C_INIT(I2C_Handle_t* I2C_HANDLE);

/**
 * @brief Generates a START condition on the I2C bus (Master mode).
 * @param pI2Cx: Pointer to I2C peripheral.
 */
void I2C_GEN_START_CONDITION(I2C_TypeDef* pI2Cx);

/**
 * @brief Generates a STOP condition on the I2C bus (Master mode).
 * @param pI2Cx: Pointer to I2C peripheral.
 */
void I2C_GEN_STOP_CONDITION(I2C_TypeDef* pI2Cx);

/**
 * @brief Enables or disables ACKing of received bytes.
 * @param pI2Cx: Pointer to I2C peripheral.
 * @param ENorDI: ENABLE or DISABLE ACK.
 */
void I2C_ACK_CTRL(I2C_TypeDef* pI2Cx, uint8_t ENorDI);

/**
 * @brief Enables or disables the I2C peripheral itself.
 * @param pI2Cx: Pointer to I2C peripheral.
 * @param ENorDI: ENABLE or DISABLE peripheral.
 */
void I2C_PERI_CTRL(I2C_TypeDef* pI2Cx, uint8_t ENorDI);

/**
 * @brief Clears the ADDR flag after address phase is complete.
 * @param pI2Cx: Pointer to I2C peripheral.
 */
void I2C_CLEAR_ADDR(I2C_TypeDef* pI2Cx);

/**
 * @brief Clears the STOPF flag after a STOP condition is detected.
 * @param pI2Cx: Pointer to I2C peripheral.
 */
void I2C_CLEAR_STOPF(I2C_TypeDef* pI2Cx);

/**
 * @brief Sends slave address with read/write direction.
 * @param pI2Cx: Pointer to I2C peripheral.
 * @param Slave_Addr: Address of target slave.
 * @param RW: I2C_READ or I2C_WRITE.
 */
void I2C_SEND_ADDR(I2C_TypeDef* pI2Cx, uint8_t Slave_Addr, uint8_t RW);

/********************************************
 * Polling-based functions
 ********************************************/
I2C_ERROR_STATUS_t I2C_MSTR_SEND_DATA(I2C_TypeDef* pI2Cx, uint8_t Slave_Addr,
                                      uint8_t* TxBuffer, uint32_t len, uint8_t Rpt_Strt);
I2C_ERROR_STATUS_t I2C_MSTR_RECEIVE_DATA(I2C_TypeDef* pI2Cx, uint8_t Slave_Addr,
                                         uint8_t* RxBuffer, uint32_t len, uint8_t Rpt_Strt);
I2C_ERROR_STATUS_t I2C_SLAVE_SEND_DATA(I2C_TypeDef* pI2Cx, uint8_t* TxBuffer,  uint32_t len);
I2C_ERROR_STATUS_t I2C_SLAVE_RECEIVE_DATA(I2C_TypeDef* pI2Cx, uint8_t* RxBuffer,  uint32_t len);

/********************************************
 * Interrupt-based functions
 ********************************************/
I2C_STATUS_t I2C_MSTR_SEND_DATA_IT(I2C_Handle_t* I2C_HANDLE, uint8_t Slave_Addr,
                                   uint8_t* TxBuffer, uint32_t len, uint8_t Rpt_Strt);
I2C_STATUS_t I2C_MSTR_RECEIVE_DATA_IT(I2C_Handle_t* I2C_HANDLE, uint8_t Slave_Addr,
                                      uint8_t* RxBuffer, uint32_t len, uint8_t Rpt_Strt);
I2C_STATUS_t I2C_SLAVE_SEND_DATA_IT(I2C_Handle_t* I2C_HANDLE, uint8_t* TxBuffer, uint32_t len);
I2C_STATUS_t I2C_SLAVE_RECEIVE_DATA_IT(I2C_Handle_t* I2C_HANDLE, uint8_t* RxBuffer, uint32_t len);

/********************************************
 * Interrupt handlers
 ********************************************/
void I2C_ITEVT_HANDLE(I2C_Handle_t* I2C_HANDLE); // Handles I2C event interrupts
void I2C_ITERR_HANDLE(I2C_Handle_t* I2C_HANDLE); // Handles I2C error interrupts

/**
 * @brief Configures NVIC interrupt for I2C.
 * @param IRQ_NUMBER: IRQ number of the I2C peripheral.
 * @param ENorDI: ENABLE or DISABLE the IRQ.
 */
void I2C_IRQ_CFG(uint8_t IRQ_NUMBER, uint8_t ENorDI);

/**
 * @brief Weak application callback function for I2C events.
 * @param APP_EV: Event type (TX_CMPLT, RX_CMPLT, BERR, ARLO, etc.)
 * User can override this in application code.
 */
__attribute__((weak)) void I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_t APP_EV);

#endif /* INC_STM32F44XX_I2C_H_ */
