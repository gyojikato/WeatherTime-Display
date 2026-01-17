/*
 * stm32f44xx_usart.c
 *
 *  Created on: May 11, 2025
 *      Author: katog
 *
 *  @brief USART driver source file for STM32F44xx
 */

#include "stm32f44xx_usart.h"

/* ============================================================
 *                      LOCAL MACROS
 * ============================================================ */

#define GET_USART_PCLK_VALUE(pUSARTx) \
	((pUSARTx == USART1 || pUSARTx == USART6) ? RCC_GET_APB2_CLK() : RCC_GET_APB1_CLK())

/* ============================================================
 *                  STATIC HELPER FUNCTIONS
 * ============================================================ */

/**
 * @brief Closes USART transmission
 */
static void usart_close_tx(USART_Handle_t *USART_HANDLE)
{
	USART_HANDLE->TX_len = 0;
	USART_HANDLE->TxBuffer = NULL;
	USART_HANDLE->USART_TX_STATUS = USART_STATUS_READY;

	USART_HANDLE->pUSARTx->CR1 &= ~USART_CR1_TXEIE_Msk;
	USART_HANDLE->pUSARTx->CR1 &= ~USART_CR1_TCIE_Msk;
}

/**
 * @brief Closes USART reception
 */
static void usart_close_rx(USART_Handle_t *USART_HANDLE)
{
	USART_HANDLE->RX_len = 0;
	USART_HANDLE->RxBuffer = NULL;
	USART_HANDLE->USART_RX_STATUS = USART_STATUS_READY;

	USART_HANDLE->pUSARTx->CR1 &= ~USART_CR1_RXNEIE_Msk;

	if (USART_HANDLE->pUSARTx->CR1 & USART_CR1_PCE_Msk)
	{
		USART_HANDLE->pUSARTx->CR1 &= ~USART_CR1_PEIE_Msk;
	}
}

/**
 * @brief Calculates and programs BRR register
 */
static void USART_BRR_CALC(USART_TypeDef *pUSARTx, USART_BAUD_RATE_t BaudRate)
{
	uint8_t over8 = (pUSARTx->CR1 >> USART_CR1_OVER8_Pos) & 0x01;
	uint32_t pclk = GET_USART_PCLK_VALUE(pUSARTx);

	uint32_t usartdiv = (25U * pclk) / ((2U - over8) * BaudRate);
	uint32_t mantissa = usartdiv / 100U;
	uint32_t fraction = usartdiv - (mantissa * 100U);

	if (over8)
	{
		pUSARTx->BRR = (mantissa << 4) | ((fraction * 8U + 50U) / 100U);
	}
	else
	{
		pUSARTx->BRR = (mantissa << 4) | ((fraction * 16U + 50U) / 100U);
	}
}

/* ============================================================
 *                  CLOCK CONTROL
 * ============================================================ */

void USART_PCLK_CTRL(USART_TypeDef *pUSARTx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		if (pUSARTx == USART1) RCC->APB2ENR |= RCC_APB2ENR_USART1EN_Msk;
		else if (pUSARTx == USART2) RCC->APB1ENR |= RCC_APB1ENR_USART2EN_Msk;
		else if (pUSARTx == USART3) RCC->APB1ENR |= RCC_APB1ENR_USART3EN_Msk;
		else if (pUSARTx == UART4)  RCC->APB1ENR |= RCC_APB1ENR_UART4EN_Msk;
		else if (pUSARTx == UART5)  RCC->APB1ENR |= RCC_APB1ENR_UART5EN_Msk;
		else if (pUSARTx == USART6) RCC->APB2ENR |= RCC_APB2ENR_USART6EN_Msk;
	}
	else
	{
		if (pUSARTx == USART1) RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN_Msk;
		else if (pUSARTx == USART2) RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN_Msk;
		else if (pUSARTx == USART3) RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN_Msk;
		else if (pUSARTx == UART4)  RCC->APB1ENR &= ~RCC_APB1ENR_UART4EN_Msk;
		else if (pUSARTx == UART5)  RCC->APB1ENR &= ~RCC_APB1ENR_UART5EN_Msk;
		else if (pUSARTx == USART6) RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN_Msk;
	}
}

/* ============================================================
 *                  INITIALIZATION
 * ============================================================ */

void USART_INIT(USART_Handle_t *USART_HANDLE)
{
	uint32_t tempreg = 0;

	USART_PCLK_CTRL(USART_HANDLE->pUSARTx, ENABLE);

	tempreg |= (USART_HANDLE->USART_MODE << USART_CR1_RE_Pos);
	tempreg |= (USART_HANDLE->USART_OVER_VAL << USART_CR1_OVER8_Pos);
	tempreg |= (USART_HANDLE->USART_WORDLEN << USART_CR1_M_Pos);

	if (USART_HANDLE->USART_PARITY_CTRL != USART_PARITY_DISABLE)
	{
		tempreg |= USART_CR1_PCE_Msk;
		if (USART_HANDLE->USART_PARITY_CTRL == USART_PARITY_ODD)
		{
			tempreg |= USART_CR1_PS_Msk;
		}
	}

	USART_HANDLE->pUSARTx->CR1 = tempreg;

	USART_BRR_CALC(USART_HANDLE->pUSARTx, USART_HANDLE->USART_BAUDRATE);
}

/* ============================================================
 *                  ENABLE / DISABLE
 * ============================================================ */

void USART_CTRL(USART_TypeDef *pUSARTx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
		pUSARTx->CR1 |= USART_CR1_UE_Msk;
	else
		pUSARTx->CR1 &= ~USART_CR1_UE_Msk;
}

/* ============================================================
 *                  BLOCKING API
 * ============================================================ */

void USART_SEND_DATA(USART_TypeDef *pUSARTx, uint8_t *TxBuffer, uint32_t len)
{
	uint8_t wordlen = (pUSARTx->CR1 >> USART_CR1_M_Pos) & 0x01;
	uint8_t parity  = (pUSARTx->CR1 >> USART_CR1_PCE_Pos) & 0x01;

	while (len--)
	{
		while (!(pUSARTx->SR & USART_SR_TXE_Msk));

		if (wordlen == USART_WORD_LEN_9BITS)
		{
			if (!parity)
			{
				pUSARTx->DR = *((uint16_t *)TxBuffer) & 0x01FF;
				TxBuffer += 2;
			}
			else
			{
				pUSARTx->DR = *TxBuffer++;
			}
		}
		else
		{
			pUSARTx->DR = parity ? (*TxBuffer++ & 0x7F) : *TxBuffer++;
		}
	}

	while (!(pUSARTx->SR & USART_SR_TC_Msk));
}

/* ============================================================
 *              INTERRUPT MODE API
 * ============================================================ */

USART_STATUS_t USART_SEND_DATA_IT(USART_Handle_t *USART_HANDLE, uint8_t *TxBuffer, uint32_t len)
{
	if (USART_HANDLE->USART_TX_STATUS != USART_STATUS_READY)
		return USART_BUSY;

	USART_HANDLE->TxBuffer = TxBuffer;
	USART_HANDLE->TX_len = len;
	USART_HANDLE->USART_TX_STATUS = USART_STATUS_TX_BUSY;

	USART_HANDLE->pUSARTx->CR1 |= USART_CR1_TXEIE_Msk;
	USART_HANDLE->pUSARTx->CR1 |= USART_CR1_TCIE_Msk;

	return USART_OK;
}

USART_STATUS_t USART_RECEIVE_DATA_IT(USART_Handle_t *USART_HANDLE, uint8_t *RxBuffer, uint32_t len)
{
	if (USART_HANDLE->USART_RX_STATUS != USART_STATUS_READY)
		return USART_BUSY;

	USART_HANDLE->RxBuffer = RxBuffer;
	USART_HANDLE->RX_len = len;
	USART_HANDLE->USART_RX_STATUS = USART_STATUS_RX_BUSY;

	USART_HANDLE->pUSARTx->CR1 |= USART_CR1_RXNEIE_Msk;

	if (USART_HANDLE->pUSARTx->CR1 & USART_CR1_PCE_Msk)
		USART_HANDLE->pUSARTx->CR1 |= USART_CR1_PEIE_Msk;

	return USART_OK;
}

/* ============================================================
 *                  IRQ HANDLER
 * ============================================================ */

void USART_IRQ_HANDLE(USART_Handle_t *USART_HANDLE)
{
	uint32_t sr = USART_HANDLE->pUSARTx->SR;

	/* TXE */
	if ((sr & USART_SR_TXE_Msk) && USART_HANDLE->USART_TX_STATUS == USART_STATUS_TX_BUSY)
	{
		if (USART_HANDLE->TX_len > 0)
		{
			if (USART_HANDLE->USART_WORDLEN == USART_WORD_LEN_9BITS &&
			    USART_HANDLE->USART_PARITY_CTRL == USART_PARITY_DISABLE)
			{
				USART_HANDLE->pUSARTx->DR =
					(*(uint16_t *)USART_HANDLE->TxBuffer) & 0x01FF;
				USART_HANDLE->TxBuffer += 2;
			}
			else
			{
				USART_HANDLE->pUSARTx->DR = *USART_HANDLE->TxBuffer++;
			}
			USART_HANDLE->TX_len--;
		}
	}

	/* TC */
	if ((sr & USART_SR_TC_Msk) && USART_HANDLE->TX_len == 0)
	{
		usart_close_tx(USART_HANDLE);
		USART_APPEV_CLLBCK(USART_TX_SUCCESS);
	}

	/* RXNE */
	if ((sr & USART_SR_RXNE_Msk) && USART_HANDLE->USART_RX_STATUS == USART_STATUS_RX_BUSY)
	{
		*USART_HANDLE->RxBuffer++ = USART_HANDLE->pUSARTx->DR;
		USART_HANDLE->RX_len--;

		if (USART_HANDLE->RX_len == 0)
		{
			usart_close_rx(USART_HANDLE);
			USART_APPEV_CLLBCK(USART_RX_SUCCESS);
		}
	}

	/* Parity error */
	if ((sr & USART_SR_PE_Msk) && (USART_HANDLE->pUSARTx->CR1 & USART_CR1_PEIE_Msk))
	{
		volatile uint8_t dummy = USART_HANDLE->pUSARTx->DR;
		(void)dummy;
		USART_APPEV_CLLBCK(USART_PE_OCCUR);
	}
}

/* ============================================================
 *                  NVIC CONTROL
 * ============================================================ */

void USART_IRQ_CFG(uint8_t IRQ_NUMBER, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
		NVIC_EnableIRQ(IRQ_NUMBER);
	else
		NVIC_DisableIRQ(IRQ_NUMBER);
}

/* ============================================================
 *                  WEAK CALLBACK
 * ============================================================ */

__attribute__((weak)) void USART_APPEV_CLLBCK(USART_APPEV_CLLBCK_t APP_EV)
{
	(void)APP_EV;
}
