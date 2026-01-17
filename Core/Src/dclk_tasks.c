/*
 * dclk_tasks.c
 *
 *  Created on: 10 Jan 2026
 *      Author: katog
 */

#include "dclk_tasks.h"

DS1307_Handle_t DS1307_DCLK_h;
SH1106_Comms_t  SH1106_comms_h;
DHT22_FUNC_t DHT22_WRAPPER_h;
SemaphoreHandle_t xI2C_BUS_Smphr;
GPIO_Handle_t I2C_DCLK_PIN_h;
SH1106_Handle_t SH1106_DCLK_h;
SemaphoreHandle_t xTIME_DATA_Smphr;
SemaphoreHandle_t xDATE_DATA_Smphr;
CFG_IT_SMPHR_t DCLK_CFG_SMPHR_h;
TimerHandle_t xTimerOneShot[MAX_TIMER_NO];
static DHT22_Handle_t DHT22_DCLK_h;
static QueueHandle_t xDHT22_DATA_Queue;
static volatile uint8_t xForceDateRST_flg; /**< Forces date re-read after time/date configuration */

TaskHandle_t xTask1; /**< vTaskDCLKInit handle */
TaskHandle_t xTask2; /**< vTaskGetTime_100ms handle */
TaskHandle_t xTask3; /**< vTaskPrintOled_Ev handle */
TaskHandle_t xTask4; /**< vTaskGetDHT22_2000ms handle */
TaskHandle_t xTask5; /**< vTaskSetTime handle */
TaskHandle_t xTask6; /**< vTaskSetDate handle */


OneShotTimerID_t xOneShotTimerID[MAX_TIMER_NO];
/* Helper Functions */
/**
 * @brief Function to aid in clearing the pending bit for EXTI
 * 		  to help in button debounce issue
 * @param	GPIO_NUMBER : the gpio pin which the button is assigned
 */
static void flushPendingBit(uint8_t GPIO_NUMBER, uint8_t IRQ_NUMBER)
{
	GPIO_IRQ_HANDLING(GPIO_NUMBER);
	NVIC_ClearPendingIRQ(IRQ_NUMBER);
}

/**
 * @brief Function that checks if the month has 31 days or not.
 * @param month: month which is to be checked (range January - December (1 - 12))
 */
static uint8_t Check31DayMonths(uint8_t month){
	uint8_t MonthWith31[] = { 1, 3, 5, 7, 8, 10, 12};

	for(uint8_t i = 0; i < 7; i++){
		if(month == MonthWith31[i]){
			return 1;
		}
	}
	return 0;
}
/**
 * @brief Function to detect if a month is february or not. Can also check leap years for february
 * @param DS1307_HANDLE: Main handle for ds1307
 */
static uint8_t retMaxDay(DS1307_Handle_t DS1307_HANDLE){
	if(DS1307_HANDLE.DS1307_DATE_HANDLE.MONTH == 2){
		if((DS1307_HANDLE.DS1307_DATE_HANDLE.YEAR % 4) == 0){
			// the year is exactly divisible by 4
			// leap year for centennial years are not calculated since
			// ds1307 range is in 0 ~ 99 only
			return 29;
		}
		else {
			return 28;
		}
	}
	else if(Check31DayMonths(DS1307_HANDLE.DS1307_DATE_HANDLE.MONTH) == 1){
		// check if given month has 31 days
		return 31;
	}
	else {
		// given month has 30 days
		return 30;
	}

}

/**
 * @brief Function to enable or disable specific tasks
 * @param status: Enable or Disable status
 */

static void EnDiOtherTasks(FunctionalState status)
{
	if(status == ENABLE){
		vTaskResume(xTask2);
		vTaskResume(xTask3);
		vTaskResume(xTask4);
	}
	else {
		vTaskSuspend(xTask2);
		vTaskSuspend(xTask3);
		vTaskSuspend(xTask4);

	}
};

/**
 * @brief Compares current and previous received time
 * @param time_a: first time handle
 * @param time_b: second time handle
 */
static uint8_t cmp_time(const DS1307_TIME_t time_a, const DS1307_TIME_t time_b )
{
	return (time_a.HOURS == time_b.HOURS && time_a.HOUR_FORMAT == time_b.HOUR_FORMAT &&
			time_a.MINUTES == time_b.MINUTES && time_a.SECONDS == time_b.SECONDS);
}

/**
 * @brief Detects day rollover (midnight) for both 12h and 24h RTC formats
 * @param ds1307_time_data: Time handle where the data is to be checked
 * @return: return 1 if time is 12:00 AM or 24:00
 */
static uint8_t check_day_reset(const DS1307_TIME_t ds1307_time_data){
	if( ds1307_time_data.HOUR_FORMAT == DS1307_12H_FORMAT_AM ){
		if( ds1307_time_data.HOURS == 12 &&
		   ds1307_time_data.MINUTES == 00 &&
		   ds1307_time_data.SECONDS == 00 ){

			return 1;
		}
	}
	else if( ds1307_time_data.HOUR_FORMAT == DS1307_24H_FORMAT ){
		if( ds1307_time_data.HOURS == 00 &&
		   ds1307_time_data.MINUTES == 00 &&
		   ds1307_time_data.SECONDS == 00 ){
			return 1;
		}
	}
	return 0;
}

/**
 * @brief Responsible for sending the screen buffer to the OLED
 * @param : none
 */
static void oled_screen_update(void)
{
	I2C_PERI_CTRL(I2C1, ENABLE);
	taskENTER_CRITICAL(); // FIXME: OLED update is blocking and long; should be moved to DMA to avoid blocking interrupts
	SH1106_UPDATE_SCRN(&SH1106_DCLK_h, SH1106_I2C_ADDR); // FIXME: Needs to be transferred to DMA so critical states can be avoided
	taskEXIT_CRITICAL();
	I2C_PERI_CTRL(I2C1, DISABLE);
}

/**
 * @brief Function that executes increment logic for time or date data during time or date configuration
 * @param pGPIOx: GPIO port which the input button lies
 * @param pin_number: GPIO pin number where the button is configured
 * @param target_val: current value of the target data to be incremented
 * @param max: maximum value of the target to be incremented
 * @param min: minimum value of the target to be incremented
 */
static void ds1307_increment(GPIO_TypeDef* pGPIOx, uint8_t pin_number, uint8_t* target_val,  uint8_t max, uint8_t min){

	vTaskDelay(pdMS_TO_TICKS(30));
	if((xSemaphoreTake(DCLK_CFG_SMPHR_h.CFG_INC_Smphr, pdMS_TO_TICKS(0))) == pdTRUE ||
			(GPIO_READ_INPUT_PIN(pGPIOx, pin_number) == 0)){
		// short delay for polling button debounce
		vTaskDelay(pdMS_TO_TICKS(30));
		(*target_val)++;
		if(*target_val > max){
			*target_val = min;
		}
	}

	if((GPIO_READ_INPUT_PIN(GPIOB, GPIO_PIN_NUMBER_3)) == 1 && DCLK_CFG_SMPHR_h.increment_flg == DISABLE){
		xTimerChangePeriod(xTimerOneShot[INC_ONE_SHOT_TIMER], pdMS_TO_TICKS(200), pdMS_TO_TICKS(20));
		DCLK_CFG_SMPHR_h.increment_flg = ENABLE;
		//TODO to be handled by software timers
	}
}


/* FreeRTOS Tasks */
/**
 * @brief FreeRTOS task to initialize the digiclock (Initializes DHT22, I2C bus and wrapper functions)
 * 			This should only run one time at init
 * @param pvParameters: FreeRTOS task parameter (not used)
 */
void vTaskDCLKInit(void* pvParameters)
{
	char initial_date_buffer[20];

	DS1307_DCLK_h.I2C_RECEIVE = DS1307_I2C_READ;
	DS1307_DCLK_h.I2C_TRANSMIT = DS1307_I2C_WRITE;

	SH1106_comms_h.SH1106_WRITE = SH1106_I2C_WRITE;

	DHT22_WRAPPER_h.DHT22_PIN_WRITE = DHT22_WRITE_PIN;
	DHT22_WRAPPER_h.DHT22_PIN_READ = DHT22_READ_PIN;
	DHT22_WRAPPER_h.DHT22_PINMODE = DHT22_PIN_MODE;
	DHT22_WRAPPER_h.DHT22_DELAY_us = DHT22_us_DELAY;
	DHT22_WRAPPER_h.get_ticks = DHTT22_CURR_TICK;

	NVIC_SetPriority(I2C1_ER_IRQn, 7);
	NVIC_SetPriority(I2C1_EV_IRQn, 6);
	NVIC_SetPriority(EXTI1_IRQn, 8);
	NVIC_SetPriority(EXTI2_IRQn, 8);
	NVIC_SetPriority(EXTI3_IRQn, 8);
	NVIC_SetPriority(EXTI15_10_IRQn, 8);

	DELAY_INIT();

	DIGILOCK_DHT22_PIN_INITS(&DHT22_PIN_h);

	I2C_DCLK_BUS_GPIO_INIT(&I2C_DCLK_PIN_h);

	I2C_DCLK_BUS_INIT(&I2C_DCLK_BUS_h);

	DHT22_INIT(&DHT22_DCLK_h, &DHT22_WRAPPER_h);

	I2C_PERI_CTRL(I2C1, ENABLE);

	DCLK_BUTTON_PINS_INIT();

	taskENTER_CRITICAL();

	SH1106_Init(&SH1106_DCLK_h, &SH1106_comms_h);

	taskEXIT_CRITICAL();

	I2C_PERI_CTRL(I2C1, DISABLE);

	xI2C_BUS_Smphr = xSemaphoreCreateBinary();

	xTIME_DATA_Smphr = xSemaphoreCreateBinary();

	xDATE_DATA_Smphr = xSemaphoreCreateBinary();

	xDHT22_DATA_Queue = xQueueCreate(2, sizeof(DHT22_BUFF_t));

	DCLK_CFG_SMPHR_h.CFG_DATE_Smphr = xSemaphoreCreateBinary();
	DCLK_CFG_SMPHR_h.CFG_TIME_Smphr = xSemaphoreCreateBinary();
	DCLK_CFG_SMPHR_h.CFG_NXT_Smphr = xSemaphoreCreateBinary();
	DCLK_CFG_SMPHR_h.CFG_INC_Smphr = xSemaphoreCreateBinary();



	// if any off the semaphores are failed to be initialized, force a system reset
	if(xI2C_BUS_Smphr == NULL || xTIME_DATA_Smphr == NULL || xDATE_DATA_Smphr == NULL
			|| xDHT22_DATA_Queue == NULL || DCLK_CFG_SMPHR_h.CFG_DATE_Smphr == NULL ||
			DCLK_CFG_SMPHR_h.CFG_TIME_Smphr == NULL || DCLK_CFG_SMPHR_h.CFG_INC_Smphr == NULL){

		NVIC_SystemReset();
	}




	I2C_PERI_CTRL(I2C1, ENABLE);
	I2C1_irq_cfg(ENABLE);
	DS1307_GET_DATE_IT(&DS1307_DCLK_h, DS1307_SLAVE_ADDR);
	if(xSemaphoreTake(xDATE_DATA_Smphr, pdMS_TO_TICKS(1000)) == pdTRUE){
		DS1307_CONVERT_RAW_DATE(&DS1307_DCLK_h.DS1307_DATE_HANDLE, DS1307_DCLK_h.raw_date);
		DateToString(&DS1307_DCLK_h.DS1307_DATE_HANDLE, initial_date_buffer, sizeof(initial_date_buffer));
		SH1106_GotoXY(&SH1106_DCLK_h, DATE_x_pos, DATE_y_pos); // coordinate position should be same as the date update event in vTaskPrintOled_Ev
		SH1106_Puts(&SH1106_DCLK_h, initial_date_buffer, &Font_7x10, SH1106_COLOR_BLACK);
	}

	xSemaphoreGive(xI2C_BUS_Smphr);

	GPIO_IRQ_CFG(EXTI1_IRQn, ENABLE);
	GPIO_IRQ_CFG(EXTI2_IRQn, ENABLE);


	vTaskDelete(NULL);

}

/**
 * @brief FreeRTOS task responsible for getting the time value from ds1307. Runs every 250ms (feel free to change period)
 * @param pvParameters: FreeRTOS task parameter (not used)
 */
void vTaskGetTime_100ms(void* pvParameters)
{
	while(1){
		if(xSemaphoreTake(xI2C_BUS_Smphr, pdMS_TO_TICKS(0)) == pdTRUE){
			I2C_PERI_CTRL(I2C1, ENABLE);
			I2C1_irq_cfg(ENABLE);
			DS1307_GET_TIME_IT(&DS1307_DCLK_h, DS1307_SLAVE_ADDR);
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	vTaskDelete(NULL);
}

/**
 * @brief FreeRTOS task responsible for printing meaningful data to the OLED target (event driven)
 * 			Also dictates whether to receive new data for Date.
 * @param pvParameters: FreeRTOS task parameter (not used)
 */
void vTaskPrintOled_Ev(void* pvParameters)
{
	while(1){

		uint8_t toled_scrn_update_ev = 0;
		DS1307_TIME_t tds1307_time_h;
		static DS1307_TIME_t tds1307_time_h_old;
		uint8_t ttraw_time[raw_time_len];
		char ttime_buffer[20];

		DHT22_BUFF_t tdht22_data_buff;

		if(xSemaphoreTake(xTIME_DATA_Smphr, pdMS_TO_TICKS(0)) == pdTRUE)
		{
			memcpy(ttraw_time, DS1307_DCLK_h.raw_time, sizeof(ttraw_time)/sizeof(ttraw_time[0])); // FIXME: double check memory here should be bytes
			DS1307_CONVERT_RAW_TIME(&tds1307_time_h, ttraw_time);
			if(cmp_time(tds1307_time_h,  tds1307_time_h_old) == 0){

				if(check_day_reset(tds1307_time_h) == 1 || xForceDateRST_flg){

					xForceDateRST_flg = 0;

					if(xSemaphoreTake(xI2C_BUS_Smphr, pdMS_TO_TICKS(100)) == pdTRUE){
						I2C_PERI_CTRL(I2C1, ENABLE);
						I2C1_irq_cfg(ENABLE);

						DS1307_GET_DATE_IT(&DS1307_DCLK_h, DS1307_SLAVE_ADDR);
					}

					if(xSemaphoreTake(xDATE_DATA_Smphr, pdMS_TO_TICKS(50)) == pdTRUE){
						uint8_t traw_date[raw_date_len];
						char tdate_bufer[20];
						DS1307_DATE_t tds1307_date_h;


						memcpy(traw_date, DS1307_DCLK_h.raw_date, sizeof(traw_date)/sizeof(traw_date[0])); // FIXME: double check memory here should be bytes
						DS1307_CONVERT_RAW_DATE(&tds1307_date_h, traw_date);
						DateToString(&tds1307_date_h, tdate_bufer, sizeof(tdate_bufer)); // FIXME: double check memory here

						SH1106_GotoXY(&SH1106_DCLK_h, DATE_x_pos, DATE_y_pos);
						SH1106_Puts(&SH1106_DCLK_h, tdate_bufer, &Font_7x10, SH1106_COLOR_BLACK);

						// store the date data to the handle
						memcpy(&DS1307_DCLK_h.DS1307_DATE_HANDLE, &tds1307_date_h, sizeof(DS1307_DCLK_h.DS1307_DATE_HANDLE));

					}

				}

				TimeToString(&tds1307_time_h, ttime_buffer, sizeof(ttime_buffer)); // FIXME: double check memory here
				SH1106_GotoXY(&SH1106_DCLK_h, TIME_x_pos, TIME_y_pos);
				SH1106_Puts(&SH1106_DCLK_h, ttime_buffer, &Font_11x18, SH1106_COLOR_BLACK);
				toled_scrn_update_ev = 1;

				// store the time data to the handle
				memcpy(&DS1307_DCLK_h.DS1307_TIME_HANDLE, &tds1307_time_h, sizeof(DS1307_DCLK_h.DS1307_TIME_HANDLE)); // TODO: Check if no error



			}
			tds1307_time_h_old = tds1307_time_h;
		}

		if(xQueueReceive(xDHT22_DATA_Queue, &tdht22_data_buff, 0) == pdTRUE){

			SH1106_GotoXY(&SH1106_DCLK_h, TEMP_x_pos, TEMP_y_pos);
			SH1106_Puts(&SH1106_DCLK_h, tdht22_data_buff.temp_buf, &Font_7x10, SH1106_COLOR_BLACK);
			SH1106_GotoXY(&SH1106_DCLK_h, HUM_x_pos, HUM_y_pos);
			SH1106_Puts(&SH1106_DCLK_h, tdht22_data_buff.hum_buf, &Font_7x10, SH1106_COLOR_BLACK);

			toled_scrn_update_ev = 1;
		}

		if(toled_scrn_update_ev){
			if(xSemaphoreTake(xI2C_BUS_Smphr, pdMS_TO_TICKS(100)) == pdTRUE){
				oled_screen_update();
				xSemaphoreGive(xI2C_BUS_Smphr);
			}
		}

		vTaskDelay(pdMS_TO_TICKS(50));
	}
	vTaskDelete(NULL); // deletes this task as this is not needed anymore
}

/**
 * @brief FreeRTOS task to receive temperature and humidity data from DHT22 Sensor (Runs every 2 seconds as per DHT22 data sheet)
 * @param pvParameters: FreeRTOS task parameter (not used)
 */
void vTaskGetDHT22_2000ms(void* pvParameters)
{
	while(1){

		DHT22_BUFF_t dht22_buffer_t;

		if(xSemaphoreTake(xI2C_BUS_Smphr, pdMS_TO_TICKS(100)) == pdTRUE)
		{
			if(DHT22_GET_TEMP_HUM(&DHT22_DCLK_h) == 1){
				int8_t integ_part;
				uint8_t frac_part;
				integ_part = (int8_t)(DHT22_DCLK_h.temperature);
				frac_part = (uint8_t)((DHT22_DCLK_h.temperature - integ_part) * 10);

				snprintf(dht22_buffer_t.temp_buf, sizeof(dht22_buffer_t.temp_buf), "T:%d.%dC", integ_part, frac_part);

				integ_part = (uint8_t)DHT22_DCLK_h.humidity;
				frac_part = (uint8_t)((DHT22_DCLK_h.humidity - integ_part) * 10);
				snprintf(dht22_buffer_t.hum_buf, sizeof(dht22_buffer_t.hum_buf), "H:%d.%d%%", integ_part, frac_part);

				xQueueSend(xDHT22_DATA_Queue, &dht22_buffer_t, pdMS_TO_TICKS(100));

			}
			xSemaphoreGive(xI2C_BUS_Smphr);
		}
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}

// static functions specifically for vTaskSetTime
/**
 * @brief Function which is first run when entering vTaskSetTime and vTaskSetDate FreeRTOS tasks. Handles the button interrupts
 * 			disables interrupts which are not needed. Forcefully Clears other pending interrupts to focus only on one of either
 * 			vTaskSetTime and vTaskSetDate FreeRTOS tasks.
 * @param
 */
static void enterTaskSetTimeDate(void)
{
	EnDiOtherTasks(DISABLE); // we dont need other tasks to run if we are setting the time
	flushPendingBit(GPIO_PIN_NUMBER_15, EXTI15_10_IRQn); // flush pending bit to avoid pre-interruption before use (de-bounce issue is also solved)
	flushPendingBit(GPIO_PIN_NUMBER_3, EXTI3_IRQn);
	DCLK_CFG_SMPHR_h.increment_flg = ENABLE;
	GPIO_IRQ_CFG(EXTI15_10_IRQn, ENABLE);
	GPIO_IRQ_CFG(EXTI3_IRQn, ENABLE);
}

/**
 * @brief Function which runs before exiting vTaskSetTime and vTaskSetDate FreeRTOS tasks. Enables again the interrupts and handles pre-pressed
 * 			button interrupts which are not needed during the execution of either vTaskSetTime and vTaskSetDate FreeRTOS tasks.
 * @param
 */
static void exitTaskSetTimeDate(void)
{
	flushPendingBit(GPIO_PIN_NUMBER_1, EXTI1_IRQn);
	flushPendingBit(GPIO_PIN_NUMBER_2, EXTI2_IRQn);
	flushPendingBit(GPIO_PIN_NUMBER_3, EXTI3_IRQn);
	flushPendingBit(GPIO_PIN_NUMBER_15, EXTI15_10_IRQn);
	GPIO_IRQ_CFG(EXTI15_10_IRQn, DISABLE);
	GPIO_IRQ_CFG(EXTI3_IRQn, DISABLE);
	GPIO_IRQ_CFG(EXTI2_IRQn, ENABLE);
	GPIO_IRQ_CFG(EXTI1_IRQn, ENABLE);
	EnDiOtherTasks(ENABLE);
}

/**
 * @brief During vTaskSetTime FreeRTOS task, this function dictates the position of the cursor in the OLED.
 * @param curr_time_setting: current time flag which is currently being configured.
 */
static void setTimeCursorPos(uint8_t curr_time_setting)
{
	const uint8_t cursor_x_pos[4] = {TIME_x_pos + (9 * 11), TIME_x_pos + (6 * 11), TIME_x_pos + (3 * 11), TIME_x_pos};
	const uint8_t cursor_y_pos[4] = {TIME_y_pos, TIME_y_pos, TIME_y_pos, TIME_y_pos};
	if(GPIO_READ_INPUT_PIN(GPIOB, GPIO_PIN_NUMBER_3) == 1){

		SH1106_Draw_Filled_Rectangle(&SH1106_DCLK_h,
				cursor_x_pos[curr_time_setting],
				cursor_y_pos[curr_time_setting],
				21,
				17,
				SH1106_COLOR_BLACK);
		// prints current cursor here if idle
		oled_screen_update(); // FIXME
		vTaskDelay(pdMS_TO_TICKS(50));
		// the cursor is covered by the time value after 50 ms
	}
}

/**
 * @brief During vTaskSetDate FreeRTOS task, this function dictates the position of the cursor in the OLED.
 * @param curr_date_setting: current date flag which is currently being configured.
 */
static void setDateCursorPos(uint8_t curr_date_setting){
	const uint8_t cursor_x_pos[4] = {set_DATE_x_pos + 77, set_DATE_x_pos + 56, set_DATE_x_pos, set_DATE_x_pos + 21};
	const uint8_t cursor_y_pos[4] = {set_DATE_y_pos, set_DATE_y_pos, set_DATE_y_pos, set_DATE_y_pos};
	if(GPIO_READ_INPUT_PIN(GPIOB, GPIO_PIN_NUMBER_3) == 1){

		SH1106_Draw_Filled_Rectangle(&SH1106_DCLK_h,
				cursor_x_pos[curr_date_setting],
				cursor_y_pos[curr_date_setting],
				curr_date_setting ? 13 : 20,
				7,
				SH1106_COLOR_BLACK);
		// prints current cursor here if idle
		oled_screen_update();
		vTaskDelay(pdMS_TO_TICKS(50));
		// the cursor is covered by the time value after 50 ms
	}
}
/**
 * @brief FreeRTOS task which configures the displayed time in the oled
 * @param pvParameters: FreeRTOS task parameter (not used)
 */
void vTaskSetTime(void* pvParameters)
{
	while(1){
		volatile uint8_t curr_time_setting = 0; // TODO values should be placed on enum
		char time_buf[20];

		if(xSemaphoreTake(DCLK_CFG_SMPHR_h.CFG_TIME_Smphr, pdMS_TO_TICKS(0)) == pdTRUE){
			if(xSemaphoreTake(xI2C_BUS_Smphr, pdMS_TO_TICKS(300)) == pdTRUE){
				SH1106_FILL_BUFFER(&SH1106_DCLK_h, SH1106_COLOR_BLACK);

				enterTaskSetTimeDate();
				vTaskSuspend(xTask6);

				while(curr_time_setting <= 3){

					switch(curr_time_setting){
					case 0:
						ds1307_increment(GPIOB,
							GPIO_PIN_NUMBER_3,
							&DS1307_DCLK_h.DS1307_TIME_HANDLE.HOUR_FORMAT,
							2,
							0);
						break;

					case 1:
						ds1307_increment(GPIOB,
							GPIO_PIN_NUMBER_3,
							&DS1307_DCLK_h.DS1307_TIME_HANDLE.SECONDS,
							59,
							0);
						break;

					case 2:
						ds1307_increment(GPIOB,
							GPIO_PIN_NUMBER_3,
							&DS1307_DCLK_h.DS1307_TIME_HANDLE.MINUTES,
							59,
							0);
						break;

					case 3:
						if(DS1307_DCLK_h.DS1307_TIME_HANDLE.HOUR_FORMAT == DS1307_24H_FORMAT){
							ds1307_increment(GPIOB,
									GPIO_PIN_NUMBER_3,
									&DS1307_DCLK_h.DS1307_TIME_HANDLE.HOURS,
									23,
									0);
						}
						else {
							ds1307_increment(GPIOB,
									GPIO_PIN_NUMBER_3,
									&DS1307_DCLK_h.DS1307_TIME_HANDLE.HOURS,
									12,
									1);
						}
						break;
					}

					if(xSemaphoreTake(DCLK_CFG_SMPHR_h.CFG_NXT_Smphr, pdMS_TO_TICKS(0))){
						curr_time_setting++;
					}

					// places the cursor to current time setting
					setTimeCursorPos(curr_time_setting);

					TimeToString(&DS1307_DCLK_h.DS1307_TIME_HANDLE, time_buf, sizeof(time_buf));
					SH1106_GotoXY(&SH1106_DCLK_h, TIME_x_pos, TIME_y_pos);
					SH1106_Puts(&SH1106_DCLK_h, time_buf, &Font_11x18, SH1106_COLOR_BLACK);
					oled_screen_update();
					vTaskDelay(pdMS_TO_TICKS(20));
				}

				// exits the while loop, reaching the end for configuration of time
				// clears the bufffer ffor the oled, exiting for configure mode
				SH1106_FILL_BUFFER(&SH1106_DCLK_h, SH1106_COLOR_BLACK);
				xForceDateRST_flg = SET;

				// set the finalized time data to the RTC here
				I2C_PERI_CTRL(I2C1, ENABLE);
				I2C1_irq_cfg(ENABLE);
				DS1307_SET_TIME_IT(&DS1307_DCLK_h, DS1307_SLAVE_ADDR);

				// perform the essential disabling and enabling of certain interrupts
				exitTaskSetTimeDate();
				vTaskResume(xTask6);


			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

/**
 * @brief FreeRTOS task which configures the displayed date in the oled
 * @param pvParameters: FreeRTOS task parameter (not used)
 */
void vTaskSetDate(void* pvParameters)
{
	while(1){
		if(xSemaphoreTake(DCLK_CFG_SMPHR_h.CFG_DATE_Smphr, pdMS_TO_TICKS(0)) == pdTRUE){
			if(xSemaphoreTake(xI2C_BUS_Smphr, pdMS_TO_TICKS(300)) == pdTRUE){

				EnDiOtherTasks(DISABLE); // we dont need other tasks to run if we are setting the time
				vTaskSuspend(xTask5);
				flushPendingBit(GPIO_PIN_NUMBER_15, EXTI15_10_IRQn); // flush pending bit to avoid pre-interruption before use (debounce issue is also solved)
				flushPendingBit(GPIO_PIN_NUMBER_3, EXTI3_IRQn);
				DCLK_CFG_SMPHR_h.increment_flg = ENABLE;
				GPIO_IRQ_CFG(EXTI15_10_IRQn, ENABLE);
				GPIO_IRQ_CFG(EXTI3_IRQn, ENABLE);

				// clear the buffer to prepare the oled for configuring mode
				SH1106_FILL_BUFFER(&SH1106_DCLK_h, SH1106_COLOR_BLACK);

				volatile  uint8_t curr_date_setting = 0;
				uint8_t max_day;
				char date_buf[20];

				while(curr_date_setting <= 3){
					switch(curr_date_setting){
					case 0:
						ds1307_increment(GPIOB,
								GPIO_PIN_NUMBER_3,
								&DS1307_DCLK_h.DS1307_DATE_HANDLE.DAY_OF_THE_WEEK,
								6,
								0);
						break;
					case 1:
						ds1307_increment(GPIOB,
							GPIO_PIN_NUMBER_3,
							&DS1307_DCLK_h.DS1307_DATE_HANDLE.YEAR,
							99,
							0);
						break;
					case 2:
						ds1307_increment(GPIOB,
							GPIO_PIN_NUMBER_3,
							&DS1307_DCLK_h.DS1307_DATE_HANDLE.MONTH,
							12,
							0);
						break;
					case 3:

						max_day = retMaxDay(DS1307_DCLK_h);
						ds1307_increment(GPIOB,
							GPIO_PIN_NUMBER_3,
							&DS1307_DCLK_h.DS1307_DATE_HANDLE.DAY,
							max_day,
							1);
						break;
					}

					if(xSemaphoreTake(DCLK_CFG_SMPHR_h.CFG_NXT_Smphr, pdMS_TO_TICKS(0) == pdTRUE)){
						curr_date_setting++;
					}

					setDateCursorPos(curr_date_setting);
					DateToString(&DS1307_DCLK_h.DS1307_DATE_HANDLE, date_buf, sizeof(date_buf));
					SH1106_GotoXY(&SH1106_DCLK_h, set_DATE_x_pos, set_DATE_y_pos);
					SH1106_Puts(&SH1106_DCLK_h, date_buf, &Font_7x10, SH1106_COLOR_BLACK);
					oled_screen_update();
					vTaskDelay(pdMS_TO_TICKS(20));

				}

				// exits the while loop, reaching the end for configuration of time
				// clears the buffer for the oled, exiting for configure mode
				SH1106_FILL_BUFFER(&SH1106_DCLK_h, SH1106_COLOR_BLACK);
				xForceDateRST_flg = SET;

				I2C_PERI_CTRL(I2C1, ENABLE);
				I2C1_irq_cfg(ENABLE);
				DS1307_SET_DATE_IT(&DS1307_DCLK_h, DS1307_SLAVE_ADDR);

				exitTaskSetTimeDate();
				vTaskResume(xTask5);

			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}


}
/* Special Functions */

/* Callback functions, helper functions which are needed to be in global scope
 * since they are either used or reference both in main and dclk_tasks.c */

/**
 * @brief One-shot software timer callback used for button debounce handling
 * @param xTimer FreeRTOS software timer handle that triggered the callback
 */
void xTimerOneShotCllBck(TimerHandle_t xTimer)
{
	OneShotTimerID_t* txTimerID;
	txTimerID = (OneShotTimerID_t*)(pvTimerGetTimerID(xTimer));
	if(txTimerID == &xOneShotTimerID[NEXT_ONE_SHOT_TIMER]){
		flushPendingBit(GPIO_PIN_NUMBER_15, EXTI15_10_IRQn);
		GPIO_IRQ_CFG(EXTI15_10_IRQn, ENABLE);
	}
	else if(txTimerID == &xOneShotTimerID[INC_ONE_SHOT_TIMER]){
		flushPendingBit(GPIO_PIN_NUMBER_3, EXTI3_IRQn);
		GPIO_IRQ_CFG(EXTI3_IRQn, ENABLE);
	}
}

/**
 * @brief Wrapper function for easy toggling of I2C interrupt configurations
 * @param ENorDI: Enable or Disable states
 */
void I2C1_irq_cfg(uint8_t ENorDI)
{
	I2C_IRQ_CFG(I2C1_ER_IRQn, ENorDI);
	I2C_IRQ_CFG(I2C1_EV_IRQn, ENorDI);
}
