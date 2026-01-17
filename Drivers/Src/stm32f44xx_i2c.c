/*
 * stm32f44xx_i2c.c
 *
 *  Created on: May 3, 2025
 *      Author: katog
 *
 *  Description: STM32F44xx I2C peripheral driver.
 *  Implements master/slave send and receive functionality, including
 *  interrupt-based and polling-based communication. Supports both
 *  Standard Mode (SM, up to 100 kHz) and Fast Mode (FM, up to 400 kHz).
 */


#include "stm32f44xx_i2c.h"

#define I2C_MAX_TIMEOUT         (0xFFFFF000) // Maximum timeout for polling loops
#define I2C_WRITE               (0)          // Write operation flag
#define I2C_READ                (1)          // Read operation flag


// ========================= Static helper functions ========================= //

/**
 * @brief Waits for a specific status flag in the I2C SR1 register to be set.
 * @param pI2Cx: Pointer to the I2C peripheral.
 * @param FLG: The flag in SR1 to wait for.
 * @return I2C_ERROR_STATUS_t: Returns timeout error if the flag is not set in time, else success.
 */
static I2C_ERROR_STATUS_t I2C_WAIT_STATUS_SR1(I2C_TypeDef* pI2Cx, uint32_t FLG)
{
	uint32_t prev_tick = DELAY_TICK();
	while(!(pI2Cx->SR1 & FLG)){
		if(DELAY_TICK() - prev_tick  > I2C_MAX_TIMEOUT){
			return I2C_ERROR_TIMEOUT;
		}
	}
	return I2C_SUCCESS;
}

/**
 * @brief Closes a receive transaction, clears buffers and resets status.
 * @param I2C_HANDLE: Pointer to the I2C handle structure.
 */
static void close_rx(I2C_Handle_t* I2C_HANDLE)
{
	I2C_HANDLE->RxLen = 0;
	I2C_HANDLE->RxSize = 0;
	I2C_HANDLE->RxBuffer = NULL;
	I2C_HANDLE->I2C_STATUS = I2C_STATUS_READY;
	I2C_HANDLE->pI2Cx->CR2 &= ~(I2C_CR2_ITBUFEN_Msk);
	I2C_HANDLE->pI2Cx->CR2 &= ~(I2C_CR2_ITEVTEN_Msk);
	I2C_HANDLE->pI2Cx->CR2 &= ~(I2C_CR2_ITERREN_Msk);
}

/**
 * @brief Closes a transmit transaction, clears buffers and resets status.
 * @param I2C_HANDLE: Pointer to the I2C handle structure.
 */
static void close_tx(I2C_Handle_t* I2C_HANDLE)
{
	I2C_HANDLE->TxLen = 0;
	I2C_HANDLE->TxSize = 0;
	I2C_HANDLE->TxBuffer = NULL;
	I2C_HANDLE->I2C_STATUS = I2C_STATUS_READY;

	// Disable I2C interrupts
	I2C_HANDLE->pI2Cx->CR2 &= ~(I2C_CR2_ITBUFEN_Msk);
	I2C_HANDLE->pI2Cx->CR2 &= ~(I2C_CR2_ITEVTEN_Msk);
	I2C_HANDLE->pI2Cx->CR2 &= ~(I2C_CR2_ITERREN_Msk);
}

// ========================= Peripheral Control Functions ========================= //

/**
 * @brief Enables or disables the peripheral clock for a given I2C peripheral.
 * @param pI2Cx: Pointer to the I2C peripheral.
 * @param ENorDI: ENABLE or DISABLE macro.
 */
void I2C_PCLK_CTRL(I2C_TypeDef* pI2Cx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE){
		if(pI2Cx == I2C1){
			RCC->APB1ENR |= (RCC_APB1ENR_I2C1EN_Msk);
		}
		else if(pI2Cx == I2C2){
			RCC->APB1ENR |= (RCC_APB1ENR_I2C2EN_Msk);
		}
		else if(pI2Cx == I2C3){
			RCC->APB1ENR |= (RCC_APB1ENR_I2C3EN_Msk);
		}
	}
	else {
		if(pI2Cx == I2C1){
			RCC->APB1ENR &= ~(RCC_APB1ENR_I2C1EN_Msk);
		}
		else if(pI2Cx == I2C2){
			RCC->APB1ENR &= ~(RCC_APB1ENR_I2C2EN_Msk);
		}
		else if(pI2Cx == I2C3){
			RCC->APB1ENR &= ~(RCC_APB1ENR_I2C3EN_Msk);
		}
	}
}

/**
 * @brief Initializes the I2C peripheral according to the handle configuration.
 * Configures timing, frequency, and mode (SM/FM).
 * @param I2C_HANDLE: Pointer to the I2C handle structure.
 */
void I2C_INIT(I2C_Handle_t* I2C_HANDLE)
{
	I2C_PCLK_CTRL(I2C_HANDLE->pI2Cx, ENABLE); // Enable clock for I2C

	uint32_t APB1_CLK = RCC_GET_APB1_CLK();
	uint8_t CR2_FREQ_VALUE = (uint8_t)(APB1_CLK / 1000000); // CR2 frequency in MHz

	// Configure CR2 register for peripheral clock frequency
	I2C_HANDLE->pI2Cx->CR2 &= ~(I2C_CR2_FREQ_Msk);
	I2C_HANDLE->pI2Cx->CR2 |= CR2_FREQ_VALUE;

	uint16_t CCR_VALUE;

	// Configure CCR and TRISE depending on speed mode
	if(I2C_HANDLE->I2C_SPD <= I2C_SPEED_SM){
		// I2C Master mode selection (SM)
		I2C_HANDLE->pI2Cx->CCR &= ~(I2C_CCR_FS_Msk);
		// maximum TRISE of standard mode is 1000ns
		I2C_HANDLE->pI2Cx->TRISE |= (CR2_FREQ_VALUE + 1) & 0x3F;

		CCR_VALUE = (uint16_t)(APB1_CLK / (2 * I2C_HANDLE->I2C_SPD));


	}
	else if(I2C_HANDLE->I2C_SPD > I2C_SPEED_SM && I2C_HANDLE->I2C_SPD <= I2C_SPEED_FM){
		// I2C Master mode selection (FM)
		I2C_HANDLE->pI2Cx->CCR |= (I2C_CCR_FS_Msk);
		// maximum TRISE of fastmode is 300ns
		I2C_HANDLE->pI2Cx->TRISE |= ((300 * (1000 / CR2_FREQ_VALUE)) + 1) & 0x3F;

		if(I2C_HANDLE->I2C_FM_DUTY_MODE == I2C_FM_DUTY_MODE_2){
			I2C_HANDLE->pI2Cx->CR2 &= ~(I2C_CCR_DUTY_Msk);
			CCR_VALUE = (uint16_t)(APB1_CLK / (3 * I2C_HANDLE->I2C_SPD));

		}
		else {
			I2C_HANDLE->pI2Cx->CR2 |= (I2C_CCR_DUTY_Msk);
			CCR_VALUE = (uint16_t)(APB1_CLK / (25 * I2C_HANDLE->I2C_SPD));

		}
	}
	else {
		return; // Invalid speed
	}
	I2C_HANDLE->pI2Cx->CCR = CCR_VALUE & 0x3FF;


}

/**
 * @brief Generates a START condition on the I2C bus.
 */
void I2C_GEN_START_CONDITION(I2C_TypeDef* pI2Cx)
{
	pI2Cx->CR1 |= (I2C_CR1_START_Msk);
}

/**
 * @brief Generates a STOP condition on the I2C bus.
 */
void I2C_GEN_STOP_CONDITION(I2C_TypeDef* pI2Cx)
{
	pI2Cx->CR1 |= (I2C_CR1_STOP_Msk);
}

/**
 * @brief
 * @param
 */
void I2C_ACK_CTRL(I2C_TypeDef* pI2Cx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE){
		pI2Cx->CR1 |= (I2C_CR1_ACK_Msk);
	}
	else {
		pI2Cx->CR1 &= ~(I2C_CR1_ACK_Msk);
	}
}


/**
 * @brief Enables or disables the I2C peripheral.
 */
void I2C_PERI_CTRL(I2C_TypeDef* pI2Cx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE){
		pI2Cx->CR1 |= (I2C_CR1_PE_Msk);
	}
	else {
		pI2Cx->CR1 &= ~(I2C_CR1_PE_Msk);
	}
}

/**
 * @brief Clears the ADDR flag in SR1/SR2 after address is sent/received.
 */
void I2C_CLEAR_ADDR(I2C_TypeDef* pI2Cx)
{
	uint32_t dummy_read;
	dummy_read = pI2Cx->SR1;
	dummy_read = pI2Cx->SR2;
	(void)dummy_read;
}

/**
 * @brief Clears the STOPF flag after a STOP condition is detected.
 */
void I2C_CLEAR_STOPF(I2C_TypeDef* pI2Cx)
{
	uint32_t dummy_read;
	dummy_read = pI2Cx->SR1;
	pI2Cx->CR1 |= 0x0000;
	(void)dummy_read;
}

/**
 * @brief Sends a 7-bit slave address with R/W bit.
 */
void I2C_SEND_ADDR(I2C_TypeDef* pI2Cx, uint8_t Slave_Addr, uint8_t RW)
{
	if(RW == I2C_WRITE){
		pI2Cx->DR = ((Slave_Addr << 0x01) & ~(0x01));
	}
	else if(RW == I2C_READ){
		pI2Cx->DR = ((Slave_Addr << 0x01) | (0x01));
	}
}


// ========================= Master/Slave Polling Functions ========================= //

/**
 * @brief Master transmits data in polling mode.
 */
I2C_ERROR_STATUS_t I2C_MSTR_SEND_DATA(I2C_TypeDef* pI2Cx, uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len, uint8_t Rpt_Strt)
{
	// Acts as a master after sending the start condition
	I2C_GEN_START_CONDITION(pI2Cx);

	// After start condition, wait until SB flag is set in the SR1 register
	if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_SB_Msk) == I2C_ERROR_TIMEOUT){
		return I2C_ERROR_TIMEOUT;
	}

	// Clear SB bit by reading SR1 and sending Slave address to the DR register
	I2C_SEND_ADDR(pI2Cx, Slave_Addr, I2C_WRITE);

	// Wait for ADDR flag to be set and then clear the ADDR flag
	if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_ADDR_Msk) == I2C_ERROR_TIMEOUT){
		return I2C_ERROR_TIMEOUT;
	}

	// clear the ADDR flag
	I2C_CLEAR_ADDR(pI2Cx);


	while(len > 0)
	{
		if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_TXE_Msk) == I2C_ERROR_TIMEOUT){
			return I2C_ERROR_TIMEOUT;
		}
		pI2Cx->DR = *TxBuffer;
		TxBuffer++;
		len--;
	}

	if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_TXE_Msk) == I2C_ERROR_TIMEOUT){
		return I2C_ERROR_TIMEOUT;
	}

	if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_BTF_Msk) == I2C_ERROR_TIMEOUT){
		return I2C_ERROR_TIMEOUT;
	}

	if(Rpt_Strt == DISABLE){
		I2C_GEN_STOP_CONDITION(pI2Cx);
	}
	return I2C_SUCCESS;
}


/**
 * @brief Master receives data in polling mode.
 */
I2C_ERROR_STATUS_t I2C_MSTR_RECEIVE_DATA(I2C_TypeDef* pI2Cx, uint8_t Slave_Addr, uint8_t* RxBuffer, uint32_t len, uint8_t Rpt_Strt)
{
	I2C_ACK_CTRL(pI2Cx, ENABLE);
	// Acts as a master after sending the start condition
	I2C_GEN_START_CONDITION(pI2Cx);

	// After start condition, wait until SB flag is set in the SR1 register
	if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_SB_Msk) == I2C_ERROR_TIMEOUT){
		return I2C_ERROR_TIMEOUT;
	}

	// Clear SB bit by reading SR1 and sending Slave address to the DR register
	I2C_SEND_ADDR(pI2Cx, Slave_Addr, I2C_READ);

	// Wait for ADDR flag to be set and then clear the ADDR flag
	if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_ADDR_Msk) == I2C_ERROR_TIMEOUT){
		return I2C_ERROR_TIMEOUT;
	}

	// case of 1 byte to be received
	if(len == 1){
		// NACK is sent before reception of byte
		I2C_ACK_CTRL(pI2Cx, DISABLE);

		// clear the ADDR flag
		I2C_CLEAR_ADDR(pI2Cx);

		if(Rpt_Strt == DISABLE){
			I2C_GEN_STOP_CONDITION(pI2Cx);
		}

		if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_RXNE_Msk) == I2C_ERROR_TIMEOUT){
			return I2C_ERROR_TIMEOUT;
		}
		*RxBuffer = pI2Cx->DR;
		len--;

		return I2C_SUCCESS;

	}

	// clear the ADDR flag
	I2C_CLEAR_ADDR(pI2Cx);

	while(len > 0){

		if(len == 1){
			I2C_ACK_CTRL(pI2Cx, DISABLE);
			if(Rpt_Strt == DISABLE){
				I2C_GEN_STOP_CONDITION(pI2Cx);
			}
		}

		if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_RXNE_Msk) == I2C_ERROR_TIMEOUT){
			return I2C_ERROR_TIMEOUT;
		}
		*RxBuffer = pI2Cx->DR;
		RxBuffer++;
		len--;
	}

	return I2C_SUCCESS;
}

// ========================= Slave Polling Functions ========================= //

/**
 * @brief Slave sends data in polling mode.
 */
I2C_ERROR_STATUS_t I2C_SLAVE_SEND_DATA(I2C_TypeDef* pI2Cx, uint8_t* TxBuffer,  uint32_t len)
{
	// wait until address bit is set, meaning slave is selected
	while(!(pI2Cx->SR1 & I2C_SR1_ADDR_Msk));
	I2C_CLEAR_ADDR(pI2Cx);

	while(len > 0){
		if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_TXE_Msk) == I2C_ERROR_TIMEOUT){
			return I2C_ERROR_TIMEOUT;
		}
		pI2Cx->DR = *TxBuffer;
		TxBuffer++;
		len--;
	}

	if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_AF_Msk) == I2C_ERROR_TIMEOUT){
		return I2C_ERROR_TIMEOUT;
	}

	// clears the AF bit after receiving a NACK
	pI2Cx->SR1 &= ~(I2C_SR1_AF_Msk);

	return I2C_SUCCESS;
}

/**
 * @brief Slave receives data in polling mode.
 */
I2C_ERROR_STATUS_t I2C_SLAVE_RECEIVE_DATA(I2C_TypeDef* pI2Cx, uint8_t* RxBuffer,  uint32_t len)
{
	// slave waits for ADDR flag to be set indicating master is talking to the correct slave
	while(!(pI2Cx->SR1 & I2C_SR1_ADDR_Msk));
	I2C_CLEAR_ADDR(pI2Cx);

	while(len > 0){
		if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_RXNE_Msk) == I2C_ERROR_TIMEOUT){
			return I2C_ERROR_TIMEOUT;
		}
		*RxBuffer = pI2Cx->DR;
		RxBuffer++;
		len--;
	}

	if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_STOPF_Msk) == I2C_ERROR_TIMEOUT){
		return I2C_ERROR_TIMEOUT;
	}
	// STOPF bit is set after successful reception from the master, clear to free SPI
	I2C_CLEAR_STOPF(pI2Cx);

	return I2C_SUCCESS;
}
// ========================= Interrupt-based Functions ========================= //
// Functions for Master/Slave communication using interrupts (non-blocking)

/**
 * @brief Initiates a non-blocking I2C Master transmit (interrupt-based).
 * @param I2C_HANDLE: Pointer to the I2C handle structure.
 * @param Slave_Addr: 7-bit address of the slave device.
 * @param TxBuffer: Pointer to the data buffer to transmit.
 * @param len: Number of bytes to transmit.
 * @param Rpt_Strt: Repeated start enable/disable (for multi-message transactions).
 * @return I2C_STATUS_t: Returns I2C_OK if transmission started, I2C_BUSY if peripheral busy.
 */
I2C_STATUS_t I2C_MSTR_SEND_DATA_IT(I2C_Handle_t* I2C_HANDLE, uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len, uint8_t Rpt_Strt)
{
	if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_READY)
	{
		I2C_HANDLE->SlaveAddress = Slave_Addr;
		I2C_HANDLE->TxBuffer = TxBuffer;
		I2C_HANDLE->TxLen = len;
		I2C_HANDLE->TxSize = len;
		I2C_HANDLE->I2C_RPT_STRT = Rpt_Strt;
		I2C_HANDLE->I2C_STATUS = I2C_STATUS_TX_BUSY;

		I2C_GEN_START_CONDITION(I2C_HANDLE->pI2Cx);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITEVTEN_Msk);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITERREN_Msk);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITBUFEN_Msk);

		return I2C_OK;
	}
	else {
		return I2C_BUSY;
	}
}

/**
 * @brief Initiates a non-blocking I2C Master receive (interrupt-based).
 * @param I2C_HANDLE: Pointer to the I2C handle structure.
 * @param Slave_Addr: 7-bit address of the slave device.
 * @param RxBuffer: Pointer to buffer to store received data.
 * @param len: Number of bytes to receive.
 * @param Rpt_Strt: Repeated start enable/disable (for multi-message transactions).
 * @return I2C_STATUS_t: Returns I2C_OK if reception started, I2C_BUSY if peripheral busy.
 */
I2C_STATUS_t I2C_MSTR_RECEIVE_DATA_IT(I2C_Handle_t* I2C_HANDLE, uint8_t Slave_Addr, uint8_t* RxBuffer, uint32_t len, uint8_t Rpt_Strt)
{
	if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_READY){
		I2C_HANDLE->SlaveAddress = Slave_Addr;
		I2C_HANDLE->RxBuffer = RxBuffer;
		I2C_HANDLE->RxLen = len;
		I2C_HANDLE->RxSize = len;
		I2C_HANDLE->I2C_RPT_STRT = Rpt_Strt;
		I2C_HANDLE->I2C_STATUS = I2C_STATUS_RX_BUSY;

		I2C_ACK_CTRL(I2C_HANDLE->pI2Cx, ENABLE);
		I2C_GEN_START_CONDITION(I2C_HANDLE->pI2Cx);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITEVTEN_Msk);
		I2C_HANDLE->pI2Cx->CR2|= (I2C_CR2_ITERREN_Msk);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITBUFEN_Msk);

		return I2C_OK;
	}
	else {
		return I2C_BUSY;
	}
}


/**
 * @brief Initiates a non-blocking I2C Slave transmit (interrupt-based).
 * @param I2C_HANDLE: Pointer to the I2C handle structure.
 * @param TxBuffer: Pointer to data buffer to transmit.
 * @param len: Number of bytes to transmit.
 * @return I2C_STATUS_t: Returns I2C_OK if transmission started, I2C_BUSY if peripheral busy.
 */
I2C_STATUS_t I2C_SLAVE_SEND_DATA_IT(I2C_Handle_t* I2C_HANDLE, uint8_t* TxBuffer, uint32_t len)
{
	if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_READY)
	{
		I2C_HANDLE->TxBuffer = TxBuffer;
		I2C_HANDLE->TxLen = len;
		I2C_HANDLE->TxSize = len;
		I2C_HANDLE->I2C_STATUS = I2C_STATUS_TX_BUSY;

		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITEVTEN_Msk);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITERREN_Msk);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITBUFEN_Msk);

		return I2C_OK;
	}
	else {
		return I2C_BUSY;
	}
}

/**
 * @brief Initiates a non-blocking I2C Slave receive (interrupt-based).
 * @param I2C_HANDLE: Pointer to the I2C handle structure.
 * @param RxBuffer: Pointer to buffer to store received data.
 * @param len: Number of bytes to receive.
 * @return I2C_STATUS_t: Returns I2C_OK if reception started, I2C_BUSY if peripheral busy.
 */
I2C_STATUS_t I2C_SLAVE_RECEIVE_DATA_IT(I2C_Handle_t* I2C_HANDLE, uint8_t* RxBuffer, uint32_t len)
{
	if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_READY){
		I2C_HANDLE->RxBuffer = RxBuffer;
		I2C_HANDLE->RxLen = len;
		I2C_HANDLE->RxSize = len;
		I2C_HANDLE->I2C_STATUS = I2C_STATUS_RX_BUSY;

		I2C_ACK_CTRL(I2C_HANDLE->pI2Cx, ENABLE);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITEVTEN_Msk);
		I2C_HANDLE->pI2Cx->CR2|= (I2C_CR2_ITERREN_Msk);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITBUFEN_Msk);

		return I2C_OK;
	}
	else {
		return I2C_BUSY;
	}
}

/**
 * @brief Handles I2C event interrupts (TXE, RXNE, BTF, STOPF, etc.).
 * @param I2C_HANDLE: Pointer to the I2C handle structure.
 */
void I2C_ITEVT_HANDLE(I2C_Handle_t* I2C_HANDLE)
{
	if(!(I2C_HANDLE->pI2Cx->CR2 & (I2C_CR2_ITEVTEN_Msk))){
		// I2C interrupt is not generated by an event proceed to error handle
		return;
	}

	uint32_t tempreg = I2C_HANDLE->pI2Cx->SR1;
	// START condition generated
	if(tempreg & I2C_SR1_SB_Msk){
		if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_TX_BUSY){

			I2C_SEND_ADDR(I2C_HANDLE->pI2Cx, I2C_HANDLE->SlaveAddress, I2C_WRITE);

		}
		else if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_RX_BUSY){

			I2C_SEND_ADDR(I2C_HANDLE->pI2Cx, I2C_HANDLE->SlaveAddress, I2C_READ);

		}
	}
	// Address phase completed
	if(tempreg & I2C_SR1_ADDR_Msk){
		if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_TX_BUSY){

			I2C_CLEAR_ADDR(I2C_HANDLE->pI2Cx);

		}
		else if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_RX_BUSY){

			if((I2C_HANDLE->RxSize == 1) && (I2C_HANDLE->pI2Cx->SR2 & I2C_SR2_MSL_Msk)){
				I2C_ACK_CTRL(I2C_HANDLE->pI2Cx, DISABLE);
				I2C_GEN_STOP_CONDITION(I2C_HANDLE->pI2Cx);
			}
			I2C_CLEAR_ADDR(I2C_HANDLE->pI2Cx);
		}
	}
	// Byte transfer finished
	if(tempreg & I2C_SR1_BTF_Msk)
	{
		if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_TX_BUSY){
			I2C_GEN_STOP_CONDITION(I2C_HANDLE->pI2Cx);
			close_tx(I2C_HANDLE);
			I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_TX_CMPLT);
		}

	}
	// STOP condition detected
	if(tempreg & I2C_SR1_STOPF_Msk){
		if(!(I2C_HANDLE->pI2Cx->SR1 & I2C_SR2_MSL_Msk)){
			I2C_CLEAR_STOPF(I2C_HANDLE->pI2Cx);
			close_rx(I2C_HANDLE);
		}
	}
	// Transmit buffer empty
	if(tempreg & I2C_SR1_TXE_Msk && I2C_HANDLE->I2C_STATUS == I2C_STATUS_TX_BUSY){
		if(I2C_HANDLE->TxLen > 0){
			I2C_HANDLE->pI2Cx->DR = *(I2C_HANDLE->TxBuffer);
			I2C_HANDLE->TxBuffer++;
			I2C_HANDLE->TxLen--;
		}
	}
	// Receive buffer not empty
	if(tempreg & I2C_SR1_RXNE_Msk && I2C_HANDLE->I2C_STATUS == I2C_STATUS_RX_BUSY){

		if(I2C_HANDLE->RxLen == 2){
			// check if device is running on master
			if(I2C_HANDLE->pI2Cx->SR2 & I2C_SR2_MSL_Msk){

				I2C_ACK_CTRL(I2C_HANDLE->pI2Cx, DISABLE);
				if(I2C_HANDLE->I2C_RPT_STRT == DISABLE){

					I2C_GEN_STOP_CONDITION(I2C_HANDLE->pI2Cx);
				}
			}
		}
		*(I2C_HANDLE->RxBuffer) = I2C_HANDLE->pI2Cx->DR;
		I2C_HANDLE->RxBuffer++;
		I2C_HANDLE->RxLen--;

		if(I2C_HANDLE->RxLen == 0){
			I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_RX_CMPLT);
			close_rx(I2C_HANDLE);
		}
	}

}

/**
 * @brief Handles I2C error interrupts (BERR, ARLO, AF, OVR, TIMEOUT, PECERR).
 * @param I2C_HANDLE: Pointer to the I2C handle structure.
 */
void I2C_ITERR_HANDLE(I2C_Handle_t* I2C_HANDLE)
{
	if(!(I2C_HANDLE->pI2Cx->CR2 & I2C_CR2_ITERREN_Msk)){
		// I2C interrupt is not due to error since I2C_CR2_ITERREN is not set
		return;
	}
	uint32_t tempreg;
	tempreg = I2C_HANDLE->pI2Cx->SR1;
	if(tempreg & I2C_SR1_BERR_Msk){
		I2C_HANDLE->pI2Cx->SR1 &= ~(I2C_SR1_BERR_Msk);
		I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_BERR_OCCUR);
	}
	if(tempreg & I2C_SR1_ARLO_Msk){
		I2C_HANDLE->pI2Cx->SR1 &= ~(I2C_SR1_ARLO_Msk);
		I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_ARLO_OCCUR);
	}
	if(tempreg & I2C_SR1_AF_Msk){
		I2C_HANDLE->pI2Cx->SR1 &= ~(I2C_SR1_AF_Msk);
		if(I2C_HANDLE->pI2Cx->SR2 & I2C_SR2_MSL_Msk)
		{
			I2C_GEN_STOP_CONDITION(I2C_HANDLE->pI2Cx);

		}
		close_tx(I2C_HANDLE);
		I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_AF_OCCUR);

	}
	if(tempreg & I2C_SR1_OVR_Msk){
		I2C_HANDLE->pI2Cx->SR1 &= ~(I2C_SR1_OVR_Msk);
		I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_OVR_OCCUR);
	}
	if(tempreg & I2C_SR1_PECERR_Msk){
		I2C_HANDLE->pI2Cx->SR1 &= ~(I2C_SR1_PECERR_Msk);
		I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_PECERR_OCCUR);
	}
	if(tempreg & I2C_SR1_TIMEOUT_Msk){
		I2C_HANDLE->pI2Cx->SR1 &= ~(I2C_SR1_TIMEOUT_Msk);
		I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_TIMEOUT_OCCUR);
	}

}

/**
 * @brief Configures I2C interrupts in NVIC.
 * @param IRQ_NUMBER: IRQ number of the I2C peripheral.
 * @param ENorDI: ENABLE to enable, DISABLE to disable the IRQ.
 */
void I2C_IRQ_CFG(uint8_t IRQ_NUMBER, uint8_t ENorDI)
{
	if(ENorDI == ENABLE){
		NVIC_EnableIRQ(IRQ_NUMBER);
	}
	else if(ENorDI == DISABLE){
		NVIC_DisableIRQ(IRQ_NUMBER);
	}
}

/**
 * @brief Weak callback function for application events.
 * @param APP_EV: Event type from I2C communication.
 * User can override this function in application code to handle events like:
 * TX_CMPLT, RX_CMPLT, errors (BERR, ARLO, AF, OVR, TIMEOUT, PECERR).
 */
__attribute__((weak)) void I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_t APP_EV){

}
