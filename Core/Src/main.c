/*
 * main.c
 *
 *  Created on: 29 Jun 2025
 *      Author: katog
 */

/**
 * @file main.c
 * @brief Application entry point and ISR definitions for the digital clock.
 *
 * Responsible for:
 *  - Creating all FreeRTOS tasks and software timers
 *  - Starting the FreeRTOS scheduler
 *  - Handling system-level interrupts (SysTick, I2C, EXTI)
 */

#include "main.h"
#include "dclk_tasks.h"
#include "ds1307_wrappers.h"

/* Task creation failure flags */
#define TASK_INIT_FAIL      (1U << 0)
#define TASK_TIME_FAIL      (1U << 1)
#define TASK_OLED_FAIL      (1U << 2)
#define TASK_DHT_FAIL       (1U << 3)
#define TASK_SET_TIME_FAIL  (1U << 4)
#define TASK_SET_DATE_FAIL  (1U << 5)

/**
 * @brief Application entry point.
 *
 * Creates all FreeRTOS tasks and software timers required by the
 * digital clock application. Forces a system reset if any critical
 * resource allocation fails.
 *
 * @return Should never return
 */
int main(void)
{
	uint16_t task_status = 0;

	if(xTaskCreate(vTaskDCLKInit,
			"vTaskDCLKInit",
			1024,
			NULL,
			5,
			&xTask1) == pdFALSE){
		task_status |= TASK_INIT_FAIL;
	}

	if(xTaskCreate(vTaskGetTime_100ms,
			"vTaskGetTime_100ms",
			1024,
			NULL,
			4,
			&xTask2) == pdFALSE){
		task_status |= TASK_TIME_FAIL;
	}

	if(xTaskCreate(vTaskPrintOled_Ev,
			"vTaskPrintOled_Ev",
			1024,
			NULL,
			3,
			&xTask3) == pdFALSE){
		task_status |= TASK_OLED_FAIL;
	}

	if(xTaskCreate(vTaskGetDHT22_2000ms,
			"vTaskGetDHT22_2000ms",
			1024,
			NULL,
			2,
			&xTask4) == pdFALSE){
		task_status |= TASK_DHT_FAIL;
	}

	if(xTaskCreate(vTaskSetTime,
			"vTaskSetTime",
			1024,
			NULL,
			1,
			&xTask5) == pdFALSE){
		task_status |= TASK_SET_TIME_FAIL;
	}

	if(xTaskCreate(vTaskSetDate,
			"vTaskSetDate",
			1024,
			NULL,
			1,
			&xTask6) == pdFALSE){
		task_status |= TASK_SET_DATE_FAIL;
	}

	for(uint8_t i = 0; i < MAX_TIMER_NO; i++){
		xTimerOneShot[i] = xTimerCreate("xTimerOneShot",
				pdMS_TO_TICKS(1000), 		// default period; updated dynamically for debounce handling
				pdFALSE,					// one-shot timer
				(void *)&xOneShotTimerID[i],
				xTimerOneShotCllBck);

		if(xTimerOneShot[i] == NULL){
			// force a reset if timers fail to allocate
			NVIC_SystemReset();
		}
	}

	if(task_status){
		// force a system reset if any task failed to be created
		NVIC_SystemReset();
	}

	vTaskStartScheduler();

	// if scheduler returns unexpectedly, force a reset
	NVIC_SystemReset();
}

/**
 * @brief SysTick interrupt handler (FreeRTOS tick source, 1 ms period)
 */
void SysTick_Handler(void)
{
	xPortSysTickHandler();
}

/**
 * @brief I2C event interrupt handler
 */
void I2C1_EV_IRQHandler(void)
{
	I2C_ITEVT_HANDLE(&I2C_DCLK_BUS_h);
}

/**
 * @brief I2C error interrupt handler
 */
void I2C1_ER_IRQHandler(void)
{
	I2C_ITERR_HANDLE(&I2C_DCLK_BUS_h);
}

/**
 * @brief EXTI interrupt handler for Date configuration button
 */
void EXTI1_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	GPIO_IRQ_HANDLING(GPIO_PIN_NUMBER_1);
	xSemaphoreGiveFromISR(DCLK_CFG_SMPHR_h.CFG_DATE_Smphr, &xHigherPriorityTaskWoken);
	GPIO_IRQ_CFG(EXTI1_IRQn, DISABLE);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief EXTI interrupt handler for Time configuration button
 */
void EXTI2_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	GPIO_IRQ_HANDLING(GPIO_PIN_NUMBER_2);
	xSemaphoreGiveFromISR(DCLK_CFG_SMPHR_h.CFG_TIME_Smphr, &xHigherPriorityTaskWoken);
	GPIO_IRQ_CFG(EXTI2_IRQn, DISABLE);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief EXTI interrupt handler for Increment button
 */
void EXTI3_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	GPIO_IRQ_HANDLING(GPIO_PIN_NUMBER_3);
	xSemaphoreGiveFromISR(DCLK_CFG_SMPHR_h.CFG_INC_Smphr, &xHigherPriorityTaskWoken);
	GPIO_IRQ_CFG(EXTI3_IRQn, DISABLE);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	DCLK_CFG_SMPHR_h.increment_flg = DISABLE;
}

/**
 * @brief EXTI interrupt handler for Confirm / Next button
 */
void EXTI15_10_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	GPIO_IRQ_HANDLING(GPIO_PIN_NUMBER_15);
	GPIO_IRQ_CFG(EXTI15_10_IRQn, DISABLE);
	xTimerChangePeriodFromISR(xTimerOneShot[NEXT_ONE_SHOT_TIMER],
	                          pdMS_TO_TICKS(100),
	                          &xHigherPriorityTaskWoken);
	xSemaphoreGiveFromISR(DCLK_CFG_SMPHR_h.CFG_NXT_Smphr, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Application-level I2C event callback.
 *
 * Handles synchronization between I2C interrupts and FreeRTOS tasks
 * for DS1307 time/date transactions.
 */
void I2C_APPEV_CLLBCK(uint8_t value)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(value == I2C_APPEV_CLLBCK_RX_CMPLT){
		if(DS1307_DCLK_h.DS1307_IT_STATUS == DS1307_IT_GET_TIME_BUSY){
			I2C1_irq_cfg(DISABLE);
			I2C_PERI_CTRL(I2C1, DISABLE);
			DS1307_DCLK_h.DS1307_IT_STATUS = DS1307_IT_FREE;
			xSemaphoreGiveFromISR(xTIME_DATA_Smphr, &xHigherPriorityTaskWoken);
			xSemaphoreGiveFromISR(xI2C_BUS_Smphr, &xHigherPriorityTaskWoken);
		}
		else if(DS1307_DCLK_h.DS1307_IT_STATUS == DS1307_IT_GET_DATE_BUSY){
			I2C1_irq_cfg(DISABLE);
			I2C_PERI_CTRL(I2C1, DISABLE);
			DS1307_DCLK_h.DS1307_IT_STATUS = DS1307_IT_FREE;
			xSemaphoreGiveFromISR(xDATE_DATA_Smphr, &xHigherPriorityTaskWoken);
			xSemaphoreGiveFromISR(xI2C_BUS_Smphr, &xHigherPriorityTaskWoken);
		}
	}

	if(value == I2C_APPEV_CLLBCK_TX_CMPLT){
		if(DS1307_DCLK_h.DS1307_IT_STATUS == DS1307_IT_SET_TIME_BUSY ||
		   DS1307_DCLK_h.DS1307_IT_STATUS == DS1307_IT_SET_DATE_BUSY){
			DS1307_DCLK_h.DS1307_IT_STATUS = DS1307_IT_FREE;
			xSemaphoreGiveFromISR(xI2C_BUS_Smphr, &xHigherPriorityTaskWoken);
			I2C1_irq_cfg(DISABLE);
			I2C_PERI_CTRL(I2C1, DISABLE);
		}
	}

	if(value == I2C_APPEV_CLLBCK_AF_OCCUR ||
	   value == I2C_APPEV_CLLBCK_ARLO_OCCUR ||
	   value == I2C_APPEV_CLLBCK_BERR_OCCUR ||
	   value == I2C_APPEV_CLLBCK_OVR_OCCUR ||
	   value == I2C_APPEV_CLLBCK_PECERR_OCCUR ||
	   value == I2C_APPEV_CLLBCK_TIMEOUT_OCCUR){
		xSemaphoreGiveFromISR(xI2C_BUS_Smphr, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
