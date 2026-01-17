/*
 * stm32f44xx_spi.c
 *
 *  Created on: Apr 20, 2025
 *      Author: katog
 *
 *  @brief SPI driver source file for STM32F44xx
 */

#include "stm32f44xx_spi.h"
#include <stddef.h>

/* ============================================================
 *                STATIC HELPER FUNCTIONS
 * ============================================================ */

/**
 * @brief Waits until a specific SPI status flag is set or timeout occurs
 * @param pSPIx Pointer to SPI peripheral
 * @param FLG   SPI status register flag to monitor
 * @retval SUCCESS if flag is set, ERROR on timeout
 */
static ErrorStatus SPI_WAIT_FLG_STATUS(SPI_TypeDef* pSPIx, uint32_t FLG)
{
	uint32_t prev_tick = DELAY_TICK();

	while(!(pSPIx->SR & FLG))
	{
		if((DELAY_TICK() - prev_tick) > SPI_MAX_TIMEOUT)
		{
			return ERROR;
		}
	}
	return SUCCESS;
}

/**
 * @brief Handles TXE interrupt event
 * @param SPI_HANDLE Pointer to SPI handle structure
 *
 * Loads transmit data into DR based on data frame format.
 * Disables TXE interrupt and notifies application when done.
 */
static void txe_handle(SPI_Handle_t* SPI_HANDLE)
{
	if(SPI_HANDLE->SPI_DFF_SIZE == SPI_DFF_8BITS)
	{
		SPI_HANDLE->pSPIx->DR = *(SPI_HANDLE->TxBuffer);
		SPI_HANDLE->TxBuffer++;
	}
	else
	{
		SPI_HANDLE->pSPIx->DR = *(uint16_t*)SPI_HANDLE->TxBuffer;
		SPI_HANDLE->TxBuffer += 2;
	}

	SPI_HANDLE->Txlen--;

	if(SPI_HANDLE->Txlen == 0)
	{
		SPI_HANDLE->TxBuffer = NULL;
		SPI_HANDLE->SPI_BUSY_STATE = SPI_READY;
		SPI_HANDLE->pSPIx->CR2 &= ~(SPI_CR2_TXEIE_Msk);
		SPI_APP_CLKBK(SPI_HANDLE, SPI_TX_CMPLT);
	}
}

/**
 * @brief Handles RXNE interrupt event
 * @param SPI_HANDLE Pointer to SPI handle structure
 *
 * Reads received data from DR and stores into receive buffer.
 * Disables RXNE interrupt and notifies application when done.
 */
static void rxne_handle(SPI_Handle_t* SPI_HANDLE)
{
	if(SPI_HANDLE->SPI_DFF_SIZE == SPI_DFF_8BITS)
	{
		*(SPI_HANDLE->RxBuffer) = SPI_HANDLE->pSPIx->DR;
		SPI_HANDLE->RxBuffer++;
	}
	else
	{
		*(uint16_t*)SPI_HANDLE->RxBuffer = SPI_HANDLE->pSPIx->DR;
		SPI_HANDLE->RxBuffer += 2;
	}

	SPI_HANDLE->Rxlen--;

	if(SPI_HANDLE->Rxlen == 0)
	{
		SPI_HANDLE->RxBuffer = NULL;
		SPI_HANDLE->SPI_BUSY_STATE = SPI_READY;
		SPI_HANDLE->pSPIx->CR2 &= ~(SPI_CR2_RXNEIE_Msk);
		SPI_APP_CLKBK(SPI_HANDLE, SPI_RX_CMPLT);
	}
}

/**
 * @brief Handles SPI overrun error
 * @param SPI_HANDLE Pointer to SPI handle structure
 *
 * Clears OVR flag by reading DR and SR.
 */
static void ovr_handle(SPI_Handle_t* SPI_HANDLE)
{
	uint32_t dummy;

	dummy = SPI_HANDLE->pSPIx->DR;
	dummy = SPI_HANDLE->pSPIx->SR;
	(void)dummy;

	SPI_APP_CLKBK(SPI_HANDLE, SPI_OVR_OCCUR);
}

/* ============================================================
 *                CLOCK & PERIPHERAL CONTROL
 * ============================================================ */

/**
 * @brief Enables or disables SPI peripheral clock
 * @param pSPIx SPI peripheral base address
 * @param ENorDI ENABLE or DISABLE
 * @retval SUCCESS if SPI instance valid, ERROR otherwise
 */
ErrorStatus SPI_PCLK_CTRL(SPI_TypeDef* pSPIx, FunctionalState ENorDI)
{
	if(ENorDI == ENABLE)
	{
		if(pSPIx == SPI1) RCC->APB2ENR |= RCC_APB2ENR_SPI1EN_Msk;
		else if(pSPIx == SPI2) RCC->APB1ENR |= RCC_APB1ENR_SPI2EN_Msk;
		else if(pSPIx == SPI3) RCC->APB1ENR |= RCC_APB1ENR_SPI3EN_Msk;
		else if(pSPIx == SPI4) RCC->APB2ENR |= RCC_APB2ENR_SPI4EN_Msk;
		else return ERROR;
	}
	else
	{
		if(pSPIx == SPI1) RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN_Msk;
		else if(pSPIx == SPI2) RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN_Msk;
		else if(pSPIx == SPI3) RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN_Msk;
		else if(pSPIx == SPI4) RCC->APB2ENR &= ~RCC_APB2ENR_SPI4EN_Msk;
		else return ERROR;
	}
	return SUCCESS;
}

/**
 * @brief Enables or disables SPI peripheral
 * @param pSPIx SPI peripheral base address
 * @param ENorDI ENABLE or DISABLE
 *
 * Waits for BUSY flag to clear before disabling SPI.
 */
void SPI_PERI_CTRL(SPI_TypeDef* pSPIx, FunctionalState ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pSPIx->CR1 |= SPI_CR1_SPE_Msk;
	}
	else
	{
		while(pSPIx->SR & SPI_SR_BSY_Msk);
		pSPIx->CR1 &= ~SPI_CR1_SPE_Msk;
	}
}

/* ============================================================
 *                SPI INITIALIZATION
 * ============================================================ */

/**
 * @brief Initializes SPI peripheral
 * @param SPI_HANDLE Pointer to SPI handle structure
 * @retval SUCCESS or ERROR
 */
ErrorStatus SPI_INIT(SPI_Handle_t* SPI_HANDLE)
{
	uint32_t tempreg = 0;

	if(SPI_PCLK_CTRL(SPI_HANDLE->pSPIx, ENABLE) == ERROR)
		return ERROR;

	tempreg = SPI_HANDLE->pSPIx->CR1;

	/* Baud rate configuration */
	if(SPI_HANDLE->SPI_BAUD_RATE == SPI_BAUD_DIV2)
		tempreg &= ~SPI_CR1_BR_Msk;
	else if(SPI_HANDLE->SPI_BAUD_RATE <= SPI_BAUD_DIV256)
		MODIFY_REG(tempreg, SPI_CR1_BR_Msk,
		           SPI_HANDLE->SPI_BAUD_RATE << SPI_CR1_BR_Pos);
	else
		return ERROR;

	/* Device mode */
	if(SPI_HANDLE->SPI_DEVICE_MODE == SPI_DEVICE_MODE_MSTR)
		tempreg |= SPI_CR1_MSTR_Msk;
	else
		tempreg &= ~SPI_CR1_MSTR_Msk;

	/* Communication mode */
	if(SPI_HANDLE->SPI_MODE == SPI_MODE_FULL_DPLX ||
	   SPI_HANDLE->SPI_MODE == SPI_MODE_SMPLX_TX)
	{
		tempreg &= ~SPI_CR1_BIDIMODE_Msk;
		tempreg &= ~SPI_CR1_RXONLY_Msk;
	}
	else if(SPI_HANDLE->SPI_MODE == SPI_MODE_HALF_DPLX)
	{
		tempreg |= SPI_CR1_BIDIMODE_Msk;
	}
	else if(SPI_HANDLE->SPI_MODE == SPI_MODE_SMPLX_RX)
	{
		tempreg &= ~SPI_CR1_BIDIMODE_Msk;
		tempreg |= SPI_CR1_RXONLY_Msk;
	}
	else return ERROR;

	/* Clock polarity & phase */
	(SPI_HANDLE->SPI_CPOL_STATE == SPI_CPOL_HI) ?
		(tempreg |= SPI_CR1_CPOL_Msk) :
		(tempreg &= ~SPI_CR1_CPOL_Msk);

	(SPI_HANDLE->SPI_CPHA_STATE == SPI_CPHA_HI) ?
		(tempreg |= SPI_CR1_CPHA_Msk) :
		(tempreg &= ~SPI_CR1_CPHA_Msk);

	/* Data frame format */
	(SPI_HANDLE->SPI_DFF_SIZE == SPI_DFF_16BITS) ?
		(tempreg |= SPI_CR1_DFF_Msk) :
		(tempreg &= ~SPI_CR1_DFF_Msk);

	/* Software slave management */
	(SPI_HANDLE->SPI_SSM_STATE == ENABLE) ?
		(tempreg |= SPI_CR1_SSM_Msk) :
		(tempreg &= ~SPI_CR1_SSM_Msk);

	SPI_HANDLE->pSPIx->CR1 = tempreg;
	return SUCCESS;
}

/* ============================================================
 *                BLOCKING DATA TRANSFER
 * ============================================================ */

/**
 * @brief Sends data using polling
 */
void SPI_SEND_DATA(SPI_TypeDef* pSPIx, uint8_t* buffer, uint32_t len)
{
	while(len--)
	{
		while(!(pSPIx->SR & SPI_SR_TXE_Msk));

		if(pSPIx->CR1 & SPI_CR1_DFF_Msk)
		{
			pSPIx->DR = *(uint16_t*)buffer;
			buffer += 2;
		}
		else
		{
			pSPIx->DR = *buffer++;
		}
	}
}

/**
 * @brief Receives data using polling
 */
void SPI_RCV_DATA(SPI_TypeDef* pSPIx, uint8_t* buffer, uint32_t len)
{
	while(len--)
	{
		if(SPI_WAIT_FLG_STATUS(pSPIx, SPI_SR_RXNE_Msk) == ERROR)
			return;

		if(pSPIx->CR1 & SPI_CR1_DFF_Msk)
		{
			*(uint16_t*)buffer = pSPIx->DR;
			buffer += 2;
		}
		else
		{
			*buffer++ = pSPIx->DR;
		}
	}
}

/* ============================================================
 *                INTERRUPT MODE TRANSFER
 * ============================================================ */

/**
 * @brief Sends data using interrupt
 */
SPI_BUSY_STATE_t SPI_SEND_DATA_IT(SPI_Handle_t* SPI_HANDLE,
                                 uint8_t* buffer, uint32_t len)
{
	if(SPI_HANDLE->SPI_BUSY_STATE != SPI_READY)
		return SPI_BUSY;

	SPI_HANDLE->TxBuffer = buffer;
	SPI_HANDLE->Txlen = len;
	SPI_HANDLE->SPI_BUSY_STATE = SPI_BUSY_IN_TX;
	SPI_HANDLE->pSPIx->CR2 |= SPI_CR2_TXEIE_Msk;

	return SPI_OK;
}

/**
 * @brief Receives data using interrupt
 */
SPI_BUSY_STATE_t SPI_RCV_DATA_IT(SPI_Handle_t* SPI_HANDLE,
                                uint8_t* buffer, uint32_t len)
{
	if(SPI_HANDLE->SPI_BUSY_STATE != SPI_READY)
		return SPI_BUSY;

	SPI_HANDLE->RxBuffer = buffer;
	SPI_HANDLE->Rxlen = len;
	SPI_HANDLE->SPI_BUSY_STATE = SPI_BUSY_IN_RX;
	SPI_HANDLE->pSPIx->CR2 |= SPI_CR2_RXNEIE_Msk;

	return SPI_OK;
}

/* ============================================================
 *                IRQ HANDLING
 * ============================================================ */

/**
 * @brief SPI interrupt service routine handler
 */
void SPI_IRQ_HANDLE(SPI_Handle_t* SPI_HANDLE)
{
	if((SPI_HANDLE->pSPIx->SR & SPI_SR_TXE_Msk) &&
	   (SPI_HANDLE->pSPIx->CR2 & SPI_CR2_TXEIE_Msk))
	{
		txe_handle(SPI_HANDLE);
	}

	if((SPI_HANDLE->pSPIx->SR & SPI_SR_RXNE_Msk) &&
	   (SPI_HANDLE->pSPIx->CR2 & SPI_CR2_RXNEIE_Msk))
	{
		rxne_handle(SPI_HANDLE);
	}

	if((SPI_HANDLE->pSPIx->SR & SPI_SR_OVR_Msk) &&
	   (SPI_HANDLE->pSPIx->CR2 & SPI_CR2_ERRIE_Msk))
	{
		ovr_handle(SPI_HANDLE);
	}
}

/* ============================================================
 *                MISC / NVIC / CALLBACK
 * ============================================================ */

/**
 * @brief Controls SSOE bit to prevent MODF errors
 */
void SSOE_BIT_CONTROL(SPI_TypeDef* pSPIx, FunctionalState ENorDI)
{
	(ENorDI == ENABLE) ?
		(pSPIx->CR2 |= SPI_CR2_SSOE_Msk) :
		(pSPIx->CR2 &= ~SPI_CR2_SSOE_Msk);
}

/**
 * @brief Enables or disables SPI interrupt in NVIC
 */
void SPI_IRQ_CFG(uint8_t IRQ_NUMBER, FunctionalState ENorDI)
{
	(ENorDI == ENABLE) ?
		NVIC_EnableIRQ(IRQ_NUMBER) :
		NVIC_DisableIRQ(IRQ_NUMBER);
}

/**
 * @brief Weak application callback function
 */
__attribute__((weak))
void SPI_APP_CLKBK(SPI_Handle_t* SPI_HANDLE, SPI_APP_EV_t SPI_APPEV)
{
	/* User implementation */
}
