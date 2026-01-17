/*
 * stm32f44xx_usart.h
 *
 *  Created on: May 11, 2025
 *      Author: katog
 *
 *  @brief USART driver header for STM32F44xx series.
 *         Provides configuration, blocking and interrupt-based data
 *         transmission/reception, and callback support.
 */

#ifndef INC_STM32F44XX_USART_H_
#define INC_STM32F44XX_USART_H_

#include "stm32f4xx.h"
#include "stm32f44xx_rcc.h"
#include <stdlib.h>

/* ============================================================
 *                          ENUMERATIONS
 * ============================================================ */

/**
 * @brief USART runtime status
 */
typedef enum {
	USART_STATUS_READY,      /*!< Peripheral ready for transmit/receive */
	USART_STATUS_TX_BUSY,    /*!< Transmitting in progress */
	USART_STATUS_RX_BUSY,    /*!< Receiving in progress */
	USART_BUSY,              /*!< Peripheral busy (generic) */
	USART_OK                 /*!< Operation completed successfully */
} USART_STATUS_t;

/**
 * @brief USART application events for callback function
 */
typedef enum {
	USART_TX_SUCCESS,        /*!< Transmission complete event */
	USART_RX_SUCCESS,        /*!< Reception complete event */
	USART_PE_OCCUR           /*!< Parity error detected */
} USART_APPEV_CLLBCK_t;

/**
 * @brief USART stop bit length selection
 */
typedef enum {
	USART_STOPBIT_LEN_1,         /*!< 1 stop bit */
	USART_STOPBIT_LEN_2,         /*!< 2 stop bits */
	USART_STOPBIT_LEN_half,      /*!< 0.5 stop bit */
	USART_STOPBIT_LEN_threehalf  /*!< 1.5 stop bit */
} USART_STOPBIT_LEN_t;

/**
 * @brief USART word length selection
 */
typedef enum {
	USART_WORD_LEN_8BITS,    /*!< 8-bit data frame */
	USART_WORD_LEN_9BITS     /*!< 9-bit data frame */
} USART_WORD_LEN_t;

/**
 * @brief USART parity configuration
 */
typedef enum {
	USART_PARITY_EVEN,       /*!< Even parity enabled */
	USART_PARITY_ODD,        /*!< Odd parity enabled */
	USART_PARITY_DISABLE     /*!< Parity disabled */
} USART_PARITY_t;

/**
 * @brief USART oversampling selection
 */
typedef enum {
	USART_OVER_16,           /*!< Oversampling by 16 */
	USART_OVER_8             /*!< Oversampling by 8 */
} USART_OVER_t;

/**
 * @brief USART mode selection
 */
typedef enum {
	USART_MODE_RX = 1,       /*!< Enable receiver only */
	USART_MODE_TX = 2,       /*!< Enable transmitter only */
	USART_MODE_RXTX = 3      /*!< Enable both transmitter and receiver */
} USART_MODE_t;

/**
 * @brief Common USART baud rates
 */
typedef enum {
	USART_BAUD_RATE_1200 = 1200,
	USART_BAUD_RATE_2400 = 2400,
	USART_BAUD_RATE_9600 = 9600,
	USART_BAUD_RATE_19200 = 19200,
	USART_BAUD_RATE_38400 = 38400,
	USART_BAUD_RATE_57600 = 57600,
	USART_BAUD_RATE_115200 = 115200,
	USART_BAUD_RATE_230400 = 230400,
	USART_BAUD_RATE_460800 = 460800,
	USART_BAUD_RATE_921600 = 921600,
	USART_BAUD_RATE_2000000 = 2000000,
	USART_BAUD_RATE_3000000 = 3000000
} USART_BAUD_RATE_t;

/* ============================================================
 *                          HANDLE STRUCTURE
 * ============================================================ */

/**
 * @brief USART handle structure
 *        Stores configuration and runtime information.
 */
typedef struct {
	USART_STOPBIT_LEN_t STOPBIT_LEN; /*!< Stop bit configuration */
	USART_WORD_LEN_t USART_WORDLEN;  /*!< Word length (8 or 9 bits) */
	USART_MODE_t USART_MODE;         /*!< USART mode (TX/RX/RXTX) */
	USART_BAUD_RATE_t USART_BAUDRATE; /*!< Baud rate */
	USART_OVER_t USART_OVER_VAL;     /*!< Oversampling mode */
	USART_PARITY_t USART_PARITY_CTRL; /*!< Parity configuration */
	USART_TypeDef* pUSARTx;          /*!< Pointer to USART peripheral */

	/* Runtime variables */
	uint32_t TX_len;                 /*!< Number of bytes left to transmit */
	uint32_t RX_len;                 /*!< Number of bytes left to receive */
	uint8_t* TxBuffer;               /*!< Pointer to TX buffer */
	uint8_t* RxBuffer;               /*!< Pointer to RX buffer */
	USART_STATUS_t USART_RX_STATUS;  /*!< RX state */
	USART_STATUS_t USART_TX_STATUS;  /*!< TX state */
} USART_Handle_t;

/* ============================================================
 *                          API FUNCTIONS
 * ============================================================ */

/**
 * @brief Enable or disable the peripheral clock for a USART
 * @param pUSARTx: Pointer to USART peripheral
 * @param ENorDI: ENABLE or DISABLE macro
 */
void USART_PCLK_CTRL(USART_TypeDef* pUSARTx, uint8_t ENorDI);

/**
 * @brief Initialize USART peripheral according to the specified parameters
 * @param USART_HANDLE: Pointer to USART handle structure
 */
void USART_INIT(USART_Handle_t* USART_HANDLE);

/**
 * @brief De-initialize USART peripheral (reset registers to default)
 * @param pUSARTx: Pointer to USART peripheral
 */
void USART_DE_INIT(USART_TypeDef* pUSARTx);

/**
 * @brief Enable or disable the USART peripheral
 * @param pUSARTx: Pointer to USART peripheral
 * @param ENorDI: ENABLE or DISABLE macro
 */
void USART_CTRL(USART_TypeDef* pUSARTx, uint8_t ENorDI);

/**
 * @brief Blocking mode data transmission
 * @param pUSARTx: Pointer to USART peripheral
 * @param TxBuffer: Pointer to data buffer
 * @param len: Number of bytes to transmit
 */
void USART_SEND_DATA(USART_TypeDef* pUSARTx, uint8_t* TxBuffer, uint32_t len);

/**
 * @brief Blocking mode data reception
 * @param pUSARTx: Pointer to USART peripheral
 * @param RxBuffer: Pointer to reception buffer
 * @param len: Number of bytes to receive
 */
void USART_RECEIVE_DATA(USART_TypeDef* pUSARTx, uint8_t* RxBuffer, uint32_t len);

/**
 * @brief Interrupt-based data transmission
 * @param USART_HANDLE: Pointer to USART handle structure
 * @param TxBuffer: Pointer to data buffer
 * @param len: Number of bytes to transmit
 * @return USART_STATUS_t: Status of the request (READY or BUSY)
 */
USART_STATUS_t USART_SEND_DATA_IT(USART_Handle_t* USART_HANDLE, uint8_t* TxBuffer, uint32_t len);

/**
 * @brief Interrupt-based data reception
 * @param USART_HANDLE: Pointer to USART handle structure
 * @param RxBuffer: Pointer to reception buffer
 * @param len: Number of bytes to receive
 * @return USART_STATUS_t: Status of the request (READY or BUSY)
 */
USART_STATUS_t USART_RECEIVE_DATA_IT(USART_Handle_t* USART_HANDLE, uint8_t* RxBuffer, uint32_t len);

/**
 * @brief Handle USART interrupts and clear flags
 * @param USART_HANDLE: Pointer to USART handle structure
 */
void USART_IRQ_HANDLE(USART_Handle_t* USART_HANDLE);

/**
 * @brief Enable or disable NVIC interrupt for USART peripheral
 * @param IRQ_NUMBER: NVIC IRQ number
 * @param ENorDI: ENABLE or DISABLE macro
 */
void USART_IRQ_CFG(uint8_t IRQ_NUMBER, uint8_t ENorDI);

/**
 * @brief Weak application callback function to handle USART events
 * @param event: Event type (TX complete, RX complete, Parity error)
 * @note Override this function in the application code
 */
__attribute__((weak)) void USART_APPEV_CLLBCK(USART_APPEV_CLLBCK_t);

#endif /* INC_STM32F44XX_USART_H_ */
