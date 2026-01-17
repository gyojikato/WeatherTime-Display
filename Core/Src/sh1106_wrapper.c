/*
 * sh1106_wrapper.c
 *
 *  Created on: 10 Jan 2026
 *      Author: katog
 */
#include "sh1106_wrapper.h"

/**
 * @brief Blocking I2C write wrapper for SH1106 OLED controller.
 *
 * This function provides a simple abstraction over the low-level I2C
 * transmit routine used by the SH1106 driver. Bus access arbitration
 * (semaphores/mutexes) must be handled externally.
 *
 * @param Slave_Addr 7-bit I2C slave address of the SH1106
 * @param TxBuffer   Pointer to transmit buffer
 * @param len        Number of bytes to transmit
 * @param Rpt_Strt   Repeated start condition control
 *
 * @return Status returned by the underlying I2C driver
 */
uint8_t SH1106_I2C_WRITE(uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len, uint8_t Rpt_Strt)
{
	return I2C_MSTR_SEND_DATA(I2C_DCLK_BUS_h.pI2Cx, Slave_Addr, TxBuffer, len, Rpt_Strt);
}
