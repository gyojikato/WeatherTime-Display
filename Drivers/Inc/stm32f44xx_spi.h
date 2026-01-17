/*
 * stm32f44xx_spi.h
 *
 *  Created on: Apr 20, 2025
 *      Author: katog
 *
 *  @brief SPI driver header file for STM32F44xx
 */

#ifndef INC_STM32F44XX_SPI_H_
#define INC_STM32F44XX_SPI_H_

#include "stm32f4xx.h"
#include "stm32f44xx_timer.h"

/* ============================================================
 *                      MACROS
 * ============================================================ */

/**
 * @brief SPI master/slave mode definitions
 */
#define SPI_SLAVE_MDOE        (0)
#define SPI_MASTER_MODE      (1)

/**
 * @brief Maximum timeout value used in polling operations
 */
#define SPI_MAX_TIMEOUT      (1000000000)

/* ============================================================
 *                      ENUMERATIONS
 * ============================================================ */

/**
 * @brief SPI device mode selection
 */
typedef enum
{
	SPI_DEVICE_MODE_MSTR,     /*!< SPI operates in master mode */
	SPI_DEVICE_MODE_SLAVE     /*!< SPI operates in slave mode  */

} SPI_DEVICE_MODE_t;

/**
 * @brief SPI communication mode selection
 */
typedef enum
{
	SPI_MODE_FULL_DPLX,       /*!< Full duplex communication */
	SPI_MODE_HALF_DPLX,       /*!< Half duplex communication */
	SPI_MODE_SMPLX_TX,        /*!< Simplex transmit only */
	SPI_MODE_SMPLX_RX         /*!< Simplex receive only */

} SPI_MODE_t;

/**
 * @brief SPI baud rate prescaler
 */
typedef enum
{
	SPI_BAUD_DIV2,
	SPI_BAUD_DIV4,
	SPI_BAUD_DIV8,
	SPI_BAUD_DIV16,
	SPI_BAUD_DIV32,
	SPI_BAUD_DIV64,
	SPI_BAUD_DIV128,
	SPI_BAUD_DIV256

} SPI_BAUD_t;

/**
 * @brief SPI data frame format size
 */
typedef enum
{
	SPI_DFF_8BITS,            /*!< 8-bit data frame */
	SPI_DFF_16BITS            /*!< 16-bit data frame */

} SPI_DFF_SIZE_t;

/**
 * @brief SPI clock polarity configuration
 */
typedef enum
{
	SPI_CPOL_HI,              /*!< Clock idle state high */
	SPI_CPOL_LO               /*!< Clock idle state low  */

} SPI_CPOL_STATE_t;

/**
 * @brief SPI clock phase configuration
 */
typedef enum
{
	SPI_CPHA_HI,              /*!< Data captured on second clock edge */
	SPI_CPHA_LO               /*!< Data captured on first clock edge  */

} SPI_CPHA_STATE_t;

/**
 * @brief SPI driver state and return status
 */
typedef enum
{
	SPI_READY,                /*!< SPI is idle and ready */
	SPI_BUSY_IN_TX,           /*!< SPI busy transmitting */
	SPI_BUSY_IN_RX,           /*!< SPI busy receiving */
	SPI_BUSY,                 /*!< SPI busy (generic) */
	SPI_OK                    /*!< Operation successful */

} SPI_BUSY_STATE_t;

/**
 * @brief SPI application callback events
 */
typedef enum
{
	SPI_TX_CMPLT,             /*!< Transmission complete */
	SPI_RX_CMPLT,             /*!< Reception complete */
	SPI_OVR_OCCUR             /*!< Overrun error occurred */

} SPI_APP_EV_t;

/* ============================================================
 *                      HANDLE STRUCTURE
 * ============================================================ */

/**
 * @brief SPI handle structure
 *
 * Holds SPI peripheral configuration, runtime state,
 * buffer pointers, and transfer lengths.
 */
typedef struct
{
	SPI_DEVICE_MODE_t   SPI_DEVICE_MODE;   /*!< Master or slave mode */
	SPI_BAUD_t          SPI_BAUD_RATE;     /*!< Baud rate prescaler */
	SPI_MODE_t          SPI_MODE;           /*!< Communication mode */
	SPI_DFF_SIZE_t      SPI_DFF_SIZE;       /*!< Data frame size */
	SPI_CPOL_STATE_t    SPI_CPOL_STATE;     /*!< Clock polarity */
	SPI_CPHA_STATE_t    SPI_CPHA_STATE;     /*!< Clock phase */
	SPI_TypeDef*        pSPIx;              /*!< SPI peripheral base address */
	SPI_BUSY_STATE_t    SPI_BUSY_STATE;     /*!< SPI current state */
	FunctionalState     SPI_SSM_STATE;      /*!< Software slave management */
	uint8_t*            RxBuffer;           /*!< Receive buffer pointer */
	uint8_t*            TxBuffer;           /*!< Transmit buffer pointer */
	uint32_t            Txlen;               /*!< Transmit length */
	uint32_t            Rxlen;               /*!< Receive length */

} SPI_Handle_t;

/* ============================================================
 *                      API PROTOTYPES
 * ============================================================ */

/**
 * @brief Enables or disables SPI peripheral clock
 * @param pSPIx Pointer to SPI peripheral base address
 * @param ENorDI ENABLE or DISABLE
 * @retval SUCCESS or ERROR
 */
ErrorStatus SPI_PCLK_CTRL(SPI_TypeDef* pSPIx, FunctionalState ENorDI);

/**
 * @brief Enables or disables SPI peripheral
 * @param pSPIx Pointer to SPI peripheral base address
 * @param ENorDI ENABLE or DISABLE
 */
void SPI_PERI_CTRL(SPI_TypeDef* pSPIx, FunctionalState ENorDI);

/**
 * @brief Initializes SPI peripheral using user configuration
 * @param SPI_HANDLE Pointer to SPI handle structure
 * @retval SUCCESS or ERROR
 */
ErrorStatus SPI_INIT(SPI_Handle_t* SPI_HANDLE);

/**
 * @brief De-initializes SPI peripheral
 * @param SPI_HANDLE Pointer to SPI handle structure
 * @retval SUCCESS or ERROR
 */
ErrorStatus SPI_DeINIT(SPI_Handle_t* SPI_HANDLE);

/**
 * @brief Sends data using polling (blocking mode)
 * @param pSPIx Pointer to SPI peripheral base address
 * @param buffer Pointer to transmit buffer
 * @param len Number of data frames to send
 */
void SPI_SEND_DATA(SPI_TypeDef* pSPIx, uint8_t* buffer, uint32_t len);

/**
 * @brief Receives data using polling (blocking mode)
 * @param pSPIx Pointer to SPI peripheral base address
 * @param buffer Pointer to receive buffer
 * @param len Number of data frames to receive
 */
void SPI_RCV_DATA(SPI_TypeDef* pSPIx, uint8_t* buffer, uint32_t len);

/**
 * @brief Receives data using interrupt mode
 * @param SPI_HANDLE Pointer to SPI handle structure
 * @param buffer Pointer to receive buffer
 * @param len Number of data frames to receive
 * @retval SPI_OK or SPI_BUSY
 */
SPI_BUSY_STATE_t SPI_RCV_DATA_IT(SPI_Handle_t* SPI_HANDLE,
                                uint8_t* buffer, uint32_t len);

/**
 * @brief Sends data using interrupt mode
 * @param SPI_HANDLE Pointer to SPI handle structure
 * @param buffer Pointer to transmit buffer
 * @param len Number of data frames to send
 * @retval SPI_OK or SPI_BUSY
 */
SPI_BUSY_STATE_t SPI_SEND_DATA_IT(SPI_Handle_t* SPI_HANDLE,
                                 uint8_t* buffer, uint32_t len);

/**
 * @brief Controls hardware slave select output (SSOE)
 * @param pSPIx Pointer to SPI peripheral base address
 * @param ENorDI ENABLE or DISABLE
 */
void SSOE_BIT_CONTROL(SPI_TypeDef* pSPIx, FunctionalState ENorDI);

/**
 * @brief Configures SPI interrupt in NVIC
 * @param IRQ_NUMBER SPI IRQ number
 * @param ENorDI ENABLE or DISABLE
 */
void SPI_IRQ_CFG(uint8_t IRQ_NUMBER, FunctionalState ENorDI);

/**
 * @brief SPI interrupt handler
 * @param SPI_HANDLE Pointer to SPI handle structure
 */
void SPI_IRQ_HANDLE(SPI_Handle_t* SPI_HANDLE);

/**
 * @brief Application callback for SPI events
 * @param SPI_HANDLE Pointer to SPI handle structure
 * @param SPI_APPEV SPI application event
 *
 * Weakly defined. User should override this function.
 */
__attribute__((weak)) void SPI_APP_CLKBK(SPI_Handle_t* SPI_HANDLE, SPI_APP_EV_t SPI_APPEV);

#endif /* INC_STM32F44XX_SPI_H_ */
