/*
 * dclk_tasks.h
 *
 *  Created on: 10 Jan 2026
 *      Author: katog
 */

/**
 * @file dclk_tasks.h
 * @brief FreeRTOS task definitions and shared resources for the digital clock application.
 *
 * This module declares all RTOS tasks, timers, semaphores, and shared handles
 * used for time/date management, OLED display updates, and sensor acquisition.
 */

#ifndef INC_DCLK_TASKS_H_
#define INC_DCLK_TASKS_H_

#include <string.h>
#include "DS1307.h"
#include "dclk_init.h"
#include "ds1307_wrappers.h"
#include "sh1106_wrapper.h"
#include "dht22_wrappers.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"


#define TIME_x_pos			(3) 	/**< X position of time string on OLED (pixels) */
#define TIME_y_pos			(23) 	/**< Y position of time string on OLED (pixels) */

#define DATE_x_pos			(15) 	/**< X position of date string on OLED (pixels) */
#define DATE_y_pos			(0) 	/**< Y position of date string on OLED (pixels) */

#define set_DATE_x_pos		DATE_x_pos 		/**< X position of date setting string on OLED (pixels) */
#define set_DATE_y_pos		(29) 			/**< Y position of date setting string on OLED (pixels) */

#define TEMP_x_pos			(0)		/**< X position of temperature string on OLED (pixels) */
#define TEMP_y_pos			(53)	/**< Y position of temperature string on OLED (pixels) */

#define HUM_x_pos			(72)	/**< X position of humidity string on OLED (pixels) */
#define HUM_y_pos			(53)	/**< Y position of humidity string on OLED (pixels) */


#define MAX_TIMER_NO		(2) 	/**< Number of one-shot timers used for button handling */


#define NEXT_ONE_SHOT_TIMER	(0)

// ID for FreeRTOS timer button for "Increment" functionality for time and date configuration
#define INC_ONE_SHOT_TIMER	(1)

typedef struct {
	SemaphoreHandle_t CFG_TIME_Smphr; /**< Semaphore for time configuration mode */
	SemaphoreHandle_t CFG_DATE_Smphr; /**< Semaphore for date configuration mode */
	SemaphoreHandle_t CFG_NXT_Smphr;  /**< Semaphore triggered by "Next" button */
	SemaphoreHandle_t CFG_INC_Smphr;  /**< Semaphore triggered by "Increment" button */
	FunctionalState  increment_flg;   /**< Indicates increment enable state */
}CFG_IT_SMPHR_t;

typedef struct {
	char temp_buf[20];  /**< holds the string for temperature data */
	char hum_buf[20];	/**< holds the string for humidity data */
}DHT22_BUFF_t;

typedef struct {
	char xTimerName[20]; /**< holds the string for FreeRTOS timer name data */
}OneShotTimerID_t;

/* Handles which are used both in main and dclk_tasks.c */
extern DS1307_Handle_t DS1307_DCLK_h;
extern SemaphoreHandle_t xI2C_BUS_Smphr;
extern SemaphoreHandle_t xTIME_DATA_Smphr;
extern SH1106_Handle_t SH1106_DCLK_h;
extern SemaphoreHandle_t xDATE_DATA_Smphr;
extern CFG_IT_SMPHR_t DCLK_CFG_SMPHR_h;
extern TimerHandle_t xTimerOneShot[MAX_TIMER_NO];

/* Task handles for FreeRTOS specific tasks */
extern TaskHandle_t xTask1; /**< vTaskDCLKInit handle */
extern TaskHandle_t xTask2; /**< vTaskGetTime_100ms handle */
extern TaskHandle_t xTask3; /**< vTaskPrintOled_Ev handle */
extern TaskHandle_t xTask4; /**< vTaskGetDHT22_2000ms handle */
extern TaskHandle_t xTask5; /**< vTaskSetTime handle */
extern TaskHandle_t xTask6; /**< vTaskSetDate handle */

/* Timer handle for FreeRTOS software timers responsible for button interrupt handling */
extern OneShotTimerID_t xOneShotTimerID[MAX_TIMER_NO];

/* FreeRTOS Tasks */
/**
 * @brief Task that is intended to run one time, calls all necessary initialization (init task)
 * @param pvParameters: FreeRTOS Task parameter
 */
void vTaskDCLKInit(void* pvParameters);

/**
 * @brief Task that retrieves Time data from DS1307 (100ms task)
 * @param pvParameters: FreeRTOS Task parameter
 */
void vTaskGetTime_100ms(void* pvParameters);

/**
 * @brief Task that is responsible for udpating the OLED screen whenever there is a change (Event driven task)
 * @param pvParameters: FreeRTOS Task parameter
 */
void vTaskPrintOled_Ev(void* pvParameters);

/**
 * @brief Task that retrieves temperature and humidity data from DHT22 (2000ms task)
 * @param pvParameters: FreeRTOS Task parameter
 */
void vTaskGetDHT22_2000ms(void* pvParameters);

/**
 * @brief Task which handles the configuration of Time data for the digiclock (Event driven task)
 * @param pvParameters: FreeRTOS Task parameter
 */
void vTaskSetTime(void* pvParameters);


/**
 * @brief Task which handles the configuration of Date data for the digiclock (Event driven task)
 * @param pvParameters: FreeRTOS Task parameter
 */
void vTaskSetDate(void* pvParameters);
/**
 * @brief Callback function for FreeRTOS one shot software timers
 * @param xTimer: Timer Handle which has the timer responsible for the callback
 */


/* Special Functions */

/* Callback functions, helper functions which are needed to be in global scope
 * since they are either used or reference both in main and dclk_tasks.c */

/**
 * @brief One-shot software timer callback used for button debounce handling
 * @param xTimer FreeRTOS software timer handle that triggered the callback
 */
void xTimerOneShotCllBck(TimerHandle_t xTimer);
/**
 * @brief Wrapper function for easy toggling of I2C interrupt configurations
 * @param ENorDI: Enable or Disable states
 */
void I2C1_irq_cfg(uint8_t ENorDI);

#endif /* INC_DCLK_TASKS_H_ */
