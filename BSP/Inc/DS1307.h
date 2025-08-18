/*
 * DS1307.h
 *
 *  Created on: Jun 1, 2025
 *      Author: katog
 */

#ifndef INC_DS1307_H_
#define INC_DS1307_H_

#include <stdint.h>
#include <stdio.h>
#define DS1307_SLAVE_ADDR			(0x68)



#define SUNDAY						(0)
#define MONDAY						(1)
#define TUESDAY						(2)
#define WEDNESDAY					(3)
#define THURSDAY					(4)
#define FRIDAY						(5)
#define SATURDAY					(6)

#define DS1307_ERROR				(0)
#define DS1307_SUCCESS				(1)

#define raw_time_len				(3)
#define raw_date_len				(4)

typedef enum {
	DS1307_24H_FORMAT,
	DS1307_12H_FORMAT_AM,
	DS1307_12H_FORMAT_PM
}HOUR_FORMAT_t;


/*typedef struct {
	uint8_t	(*I2C_TRANSMIT)(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t value);
	uint8_t (*I2C_RECEIVE)(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t* RxBuffer);
}DS1307_I2C_COMMS_t;*/

typedef struct {
 uint8_t SECONDS;
 uint8_t MINUTES;
 uint8_t HOURS;
 HOUR_FORMAT_t HOUR_FORMAT; // 24H, 12H AM, 12H PM


}DS1307_TIME_t;

typedef struct {
 uint8_t DAY;
 uint8_t DAY_OF_THE_WEEK;
 uint8_t MONTH;
 uint8_t YEAR;

}DS1307_DATE_t;

typedef enum {
	DS1307_IT_FREE,
	DS1307_IT_SET_TIME_BUSY,
	DS1307_IT_GET_TIME_BUSY,
	DS1307_IT_SET_DATE_BUSY,
	DS1307_IT_GET_DATE_BUSY

}DS1307_IT_STATUS_t;

typedef struct {

	/*DS1307_I2C_COMMS_t DS1307_COMMS_HANDLE;*/
	uint8_t	(*I2C_TRANSMIT)(uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len);
	uint8_t (*I2C_RECEIVE)(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t* RxBuffer, uint32_t len);
	DS1307_DATE_t DS1307_DATE_HANDLE;
	DS1307_TIME_t DS1307_TIME_HANDLE;
	DS1307_IT_STATUS_t DS1307_IT_STATUS;
	uint8_t raw_time[raw_time_len];
	uint8_t raw_date[raw_date_len];



}DS1307_Handle_t;

/* FUNCTION DECLARATIONS */
/**
 * @brief Function that disables the oscillator of the module and subjects the module for new time or date values
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS137_INIT(DS1307_Handle_t* DS1307_HANDLE, uint8_t SlaveAddr);

/**
 * @brief Function used to set the current time for the module
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS1307_SET_TIME(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr);

/**
 * @brief Interrupt mode Function used to set the current time for the module
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS1307_SET_TIME_IT(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr);

/**
 * @brief Function used to set the current date for the RTC module
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS1307_SET_DATE(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr);

/**
 * @brief Interrupt function used to set the current date for the RTC module
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS1307_SET_DATE_IT(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr);

/**
 * @brief Function in retrieving the current time from the module
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS1307_GET_TIME(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr);
/**
 * @brief Function that handles the processing once the DS1307_GET_TIME_IT completes receiving
 *        the burst raw 3 byte data of time
 * @param DS1307_HANDLE : Handle of ds1307 containing the raw data and usable data for time and date
 */

/**
 * @brief Interrupt mode Function in retrieving the current time from the module
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS1307_GET_TIME_IT(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr);

void DS1307_CONVERT_RAW_TIME(DS1307_TIME_t* DS1307_TIME_HANDLE, const uint8_t* raw_time);

/**
 * @brief Function in retrieving the current date from the RTC module
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS1307_GET_DATE(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr);

uint8_t DS1307_GET_DATE_IT(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr);
void DS1307_CONVERT_RAW_DATE(DS1307_DATE_t* DS1307_DATE_HANDLE, const uint8_t* raw_date);

/**
 * @brief Function to convert Date variables to string
 * @param DS1307_DATE: Handle that stores the date related values
 * @param date_holder: to hold the string equivalent of current date as a whole
 * @param date_holder_size : date_holder size in bytes
 */
void DateToString(const DS1307_DATE_t* DS1307_DATE, char *date_holder, uint32_t date_holder_size);

/**
 * @brief Function to convert Time variables to string
 * @param DS1307_TIME: Handle that stores the Time values
 * @param time_holder: output array that will hold the converted string of current time
 * @param time_holder_size : time_holder parameter size in bytes
 */
void TimeToString(const DS1307_TIME_t* DS1307_TIME, char *time_holder, uint32_t time_holder_size);

/**
 * @brief Function used to convert Day variable to string
 * @param DS1307_DATE: Handle that stores the date related values
 * @param date_holder: to hold the string equivalent of current date as a whole
 * @param day_holder_size : day_holder parameter size in bytes
 *
 */
void DayToString(const DS1307_DATE_t* DS1307_DATE, char *day_holder, uint32_t day_holder_size);

#endif /* INC_DS1307_H_ */
