/*
 * main.c
 *
 *  Created on: 29 Jun 2025
 *      Author: katog
 */

#include "main.h"

#define TIME_x_pos			(3)
#define TIME_y_pos			(23)

#define DATE_x_pos			(15) // offset to set the date info to middle
#define DATE_y_pos			(0)

#define set_DATE_x_pos		DATE_x_pos
#define set_DATE_y_pos		(29)

#define TEMP_x_pos			(0)
#define TEMP_y_pos			(53)

#define HUM_x_pos			(72)
#define HUM_y_pos			(53)

#define MAX_TIMER_NO		(2)
#define NEXT_ONE_SHOT_TIMER	(0)
#define INC_ONE_SHOT_TIMER	(1)

typedef struct {
	char temp_buf[20];
	char hum_buf[20];
}DHT22_BUFF_t;

typedef struct {
	SemaphoreHandle_t CFG_TIME_Smphr;
	SemaphoreHandle_t CFG_DATE_Smphr;
	SemaphoreHandle_t CFG_NXT_Smphr;
	SemaphoreHandle_t CFG_INC_Smphr;
	FunctionalState  increment_flg;
}CFG_IT_SMPHR_t;

typedef struct {
	char xTimerName[20];
}OneShotTimerID_t;


static I2C_Handle_t  I2C_DCLK_BUS_h;
static GPIO_Handle_t I2C_DCLK_PIN_h;
static GPIO_Handle_t DHT22_PIN_h;

static SH1106_Handle_t SH1106_DCLK_h;
static SH1106_Comms_t  SH1106_comms_h;

static DS1307_Handle_t DS1307_DCLK_h;

static DHT22_Handle_t DHT22_DCLK_h;
static DHT22_FUNC_t DHT22_WRAPPER_h;

static SemaphoreHandle_t xI2C_BUS_Smphr;
static SemaphoreHandle_t xTIME_DATA_Smphr;
static SemaphoreHandle_t xDATE_DATA_Smphr;
static QueueHandle_t xDHT22_DATA_Queue;

static TaskHandle_t xTask1;
static TaskHandle_t xTask2;
static TaskHandle_t xTask3;
static TaskHandle_t xTask4;
static TaskHandle_t xTask5;
static TaskHandle_t xTask6;

static CFG_IT_SMPHR_t DCLK_CFG_SMPHR_h;

static TimerHandle_t xTimerOneShot[MAX_TIMER_NO];
OneShotTimerID_t xOneShotTimerID[MAX_TIMER_NO];

static volatile uint8_t xForceDateRST_flg;

/* I2C BUS INIT */
/**
 * @brief
 * @param
 */
void I2C_DCLK_BUS_INIT(I2C_Handle_t* i2c_h);

/* I2C PINS INIT */
/**
 * @brief
 * @param
 */
void I2C_DCLK_BUS_GPIO_INIT(GPIO_Handle_t* gpio_h);

/* DHT22 PIN INIT */
/**
 * @brief
 * @param
 */
void DIGILOCK_DHT22_PIN_INITS(GPIO_Handle_t* DHT22_GPIO_HANDLE);

/* GPIO BUTTONS INIT */
/**
 * @brief
 * @param
 */
void DCLK_BUTTON_PINS_INIT();

/* SH1106 WRAPPER FUNCTIONS */
uint8_t SH1106_I2C_WRITE(uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len, uint8_t Rpt_Str);

/* DS1307 WRAPPER FUNCTIONS */
/**
 * @brief
 * @param
 */
uint8_t DS1307_I2C_WRITE(uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len);

/**
 * @brief
 * @param
 */
uint8_t DS1307_I2C_READ(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t* RxBuffer, uint32_t len);

/* DHT22 WRAPPER FUNCTIONS */
/**
 * @brief
 * @param
 */
void DHT22_WRITE_PIN(uint8_t value);

/**
 * @brief
 * @param
 */
uint8_t DHT22_READ_PIN(void);

/**
 * @brief
 * @param
 */
void DHT22_PIN_MODE(DHT22_PIN_MODE_t PIN_MODE);

/**
 * @brief
 * @param
 */
void DHT22_us_DELAY(uint32_t delay);

/**
 * @brief
 * @param
 */
uint32_t DHTT22_CURR_TICK(void);

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
void vTaskGetTime(void* pvParameters);

/**
 * @brief Task that is responsible for udpating the OLED screen whenever there is a change (Event driven task)
 * @param pvParameters: FreeRTOS Task parameter
 */
void vTaskPrintOled(void* pvParameters);

/**
 * @brief Task that retrieves temperature and humidity data from DHT22 (2000ms task)
 * @param pvParameters: FreeRTOS Task parameter
 */
void vTaskGetDHT22(void* pvParameters);

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
// TIMER CALLBACK FUNCTIONS

void xTimerOneShotCllBck(TimerHandle_t xTimer);

int main()
{
	uint8_t task_status = 0;

	if(xTaskCreate(vTaskDCLKInit,
			"vTaskDCLKInit",
			1024,
			NULL,
			5,
			&xTask1) == pdFALSE){
		task_status += 2;
	}

	if(xTaskCreate(vTaskGetTime,
			"vTaskGetTime",
			1024,
			NULL,
			4,
			&xTask2) == pdFALSE){
		task_status += 3;
	}

	if(xTaskCreate(vTaskPrintOled,
			"vTaskPrintOled",
			1024,
			NULL,
			3,
			&xTask3) == pdFALSE){
		task_status += 4;
	}

	if(xTaskCreate(vTaskGetDHT22,
			"vTaskGetDHT22",
			1024,
			NULL,
			2,
			&xTask4) == pdFALSE){
		task_status += 10;
	}


	if(xTaskCreate(vTaskSetTime,
			"vTaskSetTime",
			1024,
			NULL,
			1,
			&xTask5) == pdFALSE){
		task_status += 100;
	}

	if(xTaskCreate(vTaskSetDate,
			"vTaskSetDate",
			1024,
			NULL,
			1,
			&xTask6) == pdFALSE){
		task_status += 1000;
	}

	for(uint8_t i = 0; i < MAX_TIMER_NO; i++){
		xTimerOneShot[i] = xTimerCreate("xTimerOneShot",
				pdMS_TO_TICKS(1000), 		// default timer period is to be set
				pdFALSE,					// set to FALSE for One shot functionality
				(void *)&xOneShotTimerID[i],
				xTimerOneShotCllBck);
		if(xTimerOneShot[i] == NULL){ // force a reset if timers are failed to be allocated
			NVIC_SystemReset();
		}
	}

	if(task_status){
		// force a system reset if tasks are failed to be created
		NVIC_SystemReset();
	}


	vTaskStartScheduler();
	// if task scheduler starts to fail, force a reset
	NVIC_SystemReset();
}

/* Helper Functions */
/**
 * @brief Wrapper function for easy toggling of I2C interrupt configurations
 * @param ENorDI: Enable or Disable states
 */
static void I2C1_irq_cfg(uint8_t ENorDI)
{
	I2C_IRQ_CFG(I2C1_ER_IRQn, ENorDI);
	I2C_IRQ_CFG(I2C1_EV_IRQn, ENorDI);
}

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
 * @brief Responsible for sending the screen buffer to the OLED
 * @param
 */
static void oled_screen_update()
{
	I2C_PERI_CTRL(I2C1, ENABLE);
	taskENTER_CRITICAL(); // FIXME SHOULD BE DEALT WITH SINCE UPDATE SCREEN TAKES TOO LONG ne
	SH1106_UPDATE_SCRN(&SH1106_DCLK_h, SH1106_I2C_ADDR); // FIXME: Needs to be transferred to DMA so critical states can be avoided
	taskEXIT_CRITICAL();
	I2C_PERI_CTRL(I2C1, DISABLE);
}

/**
 * @brief Checks the time if the time will set a new day
 * @param
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
 * @brief
 * @param
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

/**
 * @brief
 * @param
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
 * @brief Function to aid in clearing the pending bit for EXTI
 * 		  to help in button debounce issue
 * @param	GPIO_NUMBER : the gpio pin which the button is assigned
 */
void flushPendingBit(uint8_t GPIO_NUMBER, uint8_t IRQ_NUMBER)
{
	GPIO_IRQ_HANDLING(GPIO_NUMBER);
	NVIC_ClearPendingIRQ(IRQ_NUMBER);
}

/**
 * @brief
 * @param
 */

void EnDiOtherTasks(FunctionalState status)
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
}


/* FreeRTOS Tasks */
/**
 * @brief
 * @param
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
		SH1106_GotoXY(&SH1106_DCLK_h, DATE_x_pos, DATE_y_pos); // coordinate position should be same as the date update event in vTaskPrintOled
		SH1106_Puts(&SH1106_DCLK_h, initial_date_buffer, &Font_7x10, SH1106_COLOR_BLACK);
	}

	xSemaphoreGive(xI2C_BUS_Smphr);

	GPIO_IRQ_CFG(EXTI1_IRQn, ENABLE);
	GPIO_IRQ_CFG(EXTI2_IRQn, ENABLE);


	vTaskDelete(NULL);

}

/**
 * @brief
 * @param
 */
void vTaskGetTime(void* pvParameters)
{
	while(1){
		if(xSemaphoreTake(xI2C_BUS_Smphr, pdMS_TO_TICKS(0)) == pdTRUE){
			I2C_PERI_CTRL(I2C1, ENABLE);
			I2C1_irq_cfg(ENABLE);
			DS1307_GET_TIME_IT(&DS1307_DCLK_h, DS1307_SLAVE_ADDR);
		}
		vTaskDelay(pdMS_TO_TICKS(250));
	}
	vTaskDelete(NULL);
}

/**
 * @brief
 * @param
 */
void vTaskPrintOled(void* pvParameters)
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
 * @brief
 * @param
 */
void vTaskGetDHT22(void* pvParameters)
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
static void enterTaskSetTimeDate()
{
	EnDiOtherTasks(DISABLE); // we dont need other tasks to run if we are setting the time
	flushPendingBit(GPIO_PIN_NUMBER_15, EXTI15_10_IRQn); // flush pending bit to avoid pre-interruption before use (debounce issue is also solved)
	flushPendingBit(GPIO_PIN_NUMBER_3, EXTI3_IRQn);
	DCLK_CFG_SMPHR_h.increment_flg = ENABLE;
	GPIO_IRQ_CFG(EXTI15_10_IRQn, ENABLE);
	GPIO_IRQ_CFG(EXTI3_IRQn, ENABLE);
}

static void exitTaskSetTimeDate()
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
 * @brief
 * @param
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
 * @brief
 * @param
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

				// exits the while loop
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
 * @brief
 * @param
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

				// exits the while loop
				// clears the bufffer ffor the oled, exiting for configure mode
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


/* I2C BUS INIT */
/**
 * @brief
 * @param
 */
void I2C_DCLK_BUS_INIT(I2C_Handle_t* i2c_h)
{
	i2c_h->DEVICE_ADDR 			= 0x69;
	i2c_h->I2C_SPD 				= I2C_SPEED_SM;
	i2c_h->I2C_FM_DUTY_MODE 	= I2C_FM_DUTY_MODE_2;
	i2c_h->pI2Cx 				= I2C1;
	i2c_h->I2C_STATUS 			= I2C_STATUS_READY;
	I2C_INIT(i2c_h);
}
/* I2C PINS INIT */
/**
 * @brief
 * @param
 */
void I2C_DCLK_BUS_GPIO_INIT(GPIO_Handle_t* gpio_h)
{
	gpio_h->GPIO_AFIO_MODE 		= AFIO_MODE_4;
	gpio_h->GPIO_MODE 			= GPIO_PIN_MODE_AFIO;
	gpio_h->GPIO_OUTPUT_SPD 	= GPIO_OUT_FST_SPD;
	gpio_h->GPIO_OUTPUT_TYPE 	= GPIO_OUT_OD;
	gpio_h->GPIO_PUPPD 			= GPIO_NO_PUPD;
	gpio_h->pGPIOx 				= GPIOB;

	// PB8 SCL
	gpio_h->GPIO_PIN_NUMBER 	= GPIO_PIN_NUMBER_8;
	GPIO_INIT(gpio_h);

	// PB9 SDA
	gpio_h->GPIO_PIN_NUMBER 	= GPIO_PIN_NUMBER_9;
	GPIO_INIT(gpio_h);
}

/* DHT22 PIN INIT */
/**
 * @brief
 * @param
 */
void DIGILOCK_DHT22_PIN_INITS( GPIO_Handle_t* DHT22_GPIO_HANDLE )
{
	DHT22_GPIO_HANDLE->GPIO_MODE 		= GPIO_PIN_MODE_INPUT;
	DHT22_GPIO_HANDLE->GPIO_PUPPD		= GPIO_NO_PUPD;
	DHT22_GPIO_HANDLE->GPIO_OUTPUT_TYPE = GPIO_OUT_PP;
	DHT22_GPIO_HANDLE->GPIO_OUTPUT_SPD	= GPIO_OUT_FST_SPD;
	DHT22_GPIO_HANDLE->pGPIOx 			= GPIOC;
	DHT22_GPIO_HANDLE->GPIO_PIN_NUMBER	= GPIO_PIN_NUMBER_9;

	GPIO_INIT( DHT22_GPIO_HANDLE );
}

/* GPIO BUTTONS INIT */
/**
 * @brief
 * @param
 */
void DCLK_BUTTON_PINS_INIT()
{
	GPIO_Handle_t DCLK_BTTNS;
	// increment and confirm buttons
	DCLK_BTTNS.GPIO_MODE = GPIO_PIN_MODE_INPUT;
	DCLK_BTTNS.GPIO_PUPPD = GPIO_PULL_UP;
	DCLK_BTTNS.pGPIOx = GPIOB;
	DCLK_BTTNS.EXTI_CFG.EXTI_EMR_SET = DISABLE;
	DCLK_BTTNS.EXTI_CFG.EXTI_IMR_SET = ENABLE;
	DCLK_BTTNS.EXTI_CFG.EXTI_FTSR_SEL = ENABLE;
	DCLK_BTTNS.EXTI_CFG.EXTI_RTSR_SEL = DISABLE;

	// EXTI to enter vTaskSetDate task
	DCLK_BTTNS.GPIO_PIN_NUMBER = GPIO_PIN_NUMBER_1;
	GPIO_INIT(&DCLK_BTTNS);

	// EXTI to enter vTaskSetTime task
	DCLK_BTTNS.GPIO_PIN_NUMBER = GPIO_PIN_NUMBER_2;
	GPIO_INIT(&DCLK_BTTNS);
	/*EXTI[0].IMR &= ~(1 << GPIO_PIN_NUMBER_2);*/

	// EXTI for increment
	DCLK_BTTNS.GPIO_PIN_NUMBER = GPIO_PIN_NUMBER_3;
	GPIO_INIT(&DCLK_BTTNS);

	// EXTI for confirm button
	DCLK_BTTNS.GPIO_PIN_NUMBER = GPIO_PIN_NUMBER_15;
	GPIO_INIT(&DCLK_BTTNS);

}

/* SH1106 WRAPPER FUNCTION */
/**
 * @brief
 * @param
 */
uint8_t SH1106_I2C_WRITE(uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len, uint8_t Rpt_Strt)
{
	return I2C_MSTR_SEND_DATA(I2C_DCLK_BUS_h.pI2Cx, Slave_Addr, TxBuffer, len, Rpt_Strt);
}

/* DS1307 WRAPPER FUNCTIONS */
/**
 * @brief
 * @param
 */
uint8_t DS1307_I2C_WRITE(uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len)
{
	TimeOut_t xTimeOut;
	TickType_t ticks_to_wait = pdMS_TO_TICKS(100);
	vTaskSetTimeOutState(&xTimeOut);
	while(I2C_MSTR_SEND_DATA_IT(&I2C_DCLK_BUS_h, Slave_Addr, TxBuffer, len, DISABLE) == I2C_BUSY){
		if(xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE){
			return 0;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	return 1;
}

/**
 * @brief
 * @param
 */
uint8_t DS1307_I2C_READ(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t* RxBuffer, uint32_t len)
{
	TimeOut_t xTimeOut;
	TickType_t ticks_to_wait = pdMS_TO_TICKS(300);
	vTaskSetTimeOutState(&xTimeOut);
	while(I2C_MSTR_SEND_DATA_IT(&I2C_DCLK_BUS_h, Slave_Addr, &reg_addr, 1, ENABLE) == I2C_BUSY){
		if(xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE){

			return 0;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	vTaskSetTimeOutState(&xTimeOut);
	while(I2C_MSTR_RECEIVE_DATA_IT(&I2C_DCLK_BUS_h, Slave_Addr, RxBuffer, len, DISABLE) == I2C_BUSY){
		if(xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE){

			return 0;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	return 1;
}

/* DHT22 WRAPPER FUNCTIONS */
/**
 * @brief
 * @param
 */
void DHT22_WRITE_PIN(uint8_t value)
{
	GPIO_WRITE_OUTPUT_PIN(GPIOC, GPIO_PIN_NUMBER_9, value);
}

/**
 * @brief
 * @param
 */
uint8_t DHT22_READ_PIN(void)
{
	return GPIO_READ_INPUT_PIN(GPIOC, GPIO_PIN_NUMBER_9);
}

/**
 * @brief
 * @param
 */
void DHT22_PIN_MODE(DHT22_PIN_MODE_t PIN_MODE)
{
	if(PIN_MODE == DHT22_PIN_MODE_INPUT){
		DHT22_PIN_h.GPIO_MODE = GPIO_PIN_MODE_INPUT;
	}
	else {
		DHT22_PIN_h.GPIO_MODE = GPIO_PIN_MODE_OUTPUT;
	}
	GPIO_INIT(&DHT22_PIN_h);
}

/**
 * @brief
 * @param
 */
void DHT22_us_DELAY(uint32_t delay)
{
	DELAY_us(delay);
}

/**
 * @brief
 * @param
 */
uint32_t DHTT22_CURR_TICK(void)
{
	return DELAY_TICK();
}

/**
 * @brief
 * @param
 */
void SysTick_Handler(void)
{
    xPortSysTickHandler();  // FreeRTOS tick handler
}

/**
 * @brief
 * @param
 */
void I2C1_EV_IRQHandler(void)
{
	I2C_ITEVT_HANDLE(&I2C_DCLK_BUS_h);
}

/**
 * @brief
 * @param
 */
void I2C1_ER_IRQHandler(void)
{
	I2C_ITERR_HANDLE(&I2C_DCLK_BUS_h);
}

/**
 * @brief
 * @param
 */
void EXTI1_IRQHandler(void)
{
	// interrupt for date
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	GPIO_IRQ_HANDLING(GPIO_PIN_NUMBER_1);
	xSemaphoreGiveFromISR(DCLK_CFG_SMPHR_h.CFG_DATE_Smphr, &xHigherPriorityTaskWoken);
	GPIO_IRQ_CFG(EXTI1_IRQn, DISABLE);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

/**
 * @brief
 * @param
 */
void EXTI2_IRQHandler(void)
{
	// interrupt for time
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	GPIO_IRQ_HANDLING(GPIO_PIN_NUMBER_2);
	xSemaphoreGiveFromISR(DCLK_CFG_SMPHR_h.CFG_TIME_Smphr, &xHigherPriorityTaskWoken);
	GPIO_IRQ_CFG(EXTI2_IRQn, DISABLE);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

/**
 * @brief
 * @param
 */
void EXTI3_IRQHandler()
{
	// Interrupt for increment
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	GPIO_IRQ_HANDLING(GPIO_PIN_NUMBER_3);
	xSemaphoreGiveFromISR(DCLK_CFG_SMPHR_h.CFG_INC_Smphr, &xHigherPriorityTaskWoken);
	GPIO_IRQ_CFG(EXTI3_IRQn, DISABLE);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	DCLK_CFG_SMPHR_h.increment_flg = DISABLE;
}

/**
 * @brief
 * @param
 */
void EXTI15_10_IRQHandler(void)
{
	// interrupt for confirm button
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	GPIO_IRQ_HANDLING(GPIO_PIN_NUMBER_15);
	GPIO_IRQ_CFG(EXTI15_10_IRQn, DISABLE);
	xTimerChangePeriodFromISR(xTimerOneShot[NEXT_ONE_SHOT_TIMER], pdMS_TO_TICKS(100), &xHigherPriorityTaskWoken);
	xSemaphoreGiveFromISR(DCLK_CFG_SMPHR_h.CFG_NXT_Smphr, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

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
 * @brief
 * @param
 */
void I2C_APPEV_CLLBCK(uint8_t value)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(value == I2C_APPEV_CLLBCK_RX_CMPLT){
		if(DS1307_DCLK_h.DS1307_IT_STATUS == DS1307_IT_GET_TIME_BUSY){
			I2C1_irq_cfg(DISABLE);
			I2C_PERI_CTRL(I2C1, DISABLE);
			DS1307_DCLK_h.DS1307_IT_STATUS = DS1307_IT_FREE;
			xSemaphoreGiveFromISR(xTIME_DATA_Smphr, &xHigherPriorityTaskWoken); // signals the tasks for new time data
			xSemaphoreGiveFromISR(xI2C_BUS_Smphr, &xHigherPriorityTaskWoken); // free the semaphore for other I2C transactions
		}
		else if(DS1307_DCLK_h.DS1307_IT_STATUS == DS1307_IT_GET_DATE_BUSY){
			I2C1_irq_cfg(DISABLE);
			I2C_PERI_CTRL(I2C1, DISABLE);
			DS1307_DCLK_h.DS1307_IT_STATUS = DS1307_IT_FREE;
			xSemaphoreGiveFromISR(xDATE_DATA_Smphr, &xHigherPriorityTaskWoken); // signals the task for new date data
			xSemaphoreGiveFromISR(xI2C_BUS_Smphr, &xHigherPriorityTaskWoken); // free the semaphore for other I2C transactions
		}
	}

	if(value == I2C_APPEV_CLLBCK_TX_CMPLT){
		if(DS1307_DCLK_h.DS1307_IT_STATUS == DS1307_IT_SET_TIME_BUSY || DS1307_DCLK_h.DS1307_IT_STATUS == DS1307_IT_SET_DATE_BUSY){
			DS1307_DCLK_h.DS1307_IT_STATUS = DS1307_IT_FREE;
			xSemaphoreGiveFromISR(xI2C_BUS_Smphr, &xHigherPriorityTaskWoken); // free the semaphore for other I2C transactions
			I2C1_irq_cfg(DISABLE);
			I2C_PERI_CTRL(I2C1, DISABLE);
		}
	}

	if(value == I2C_APPEV_CLLBCK_AF_OCCUR || value == I2C_APPEV_CLLBCK_ARLO_OCCUR ||
			value == I2C_APPEV_CLLBCK_BERR_OCCUR || value == I2C_APPEV_CLLBCK_OVR_OCCUR ||
			value == I2C_APPEV_CLLBCK_PECERR_OCCUR || value == I2C_APPEV_CLLBCK_TIMEOUT_OCCUR){
		xSemaphoreGiveFromISR(xI2C_BUS_Smphr, &xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
