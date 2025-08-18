/*
 * DS1307.h
 *
 *  Created on: Jun 1, 2025
 *      Author: katog
 */


// DONT FORGET TO CONNECT THE MODULE TO LOGIC LEVEL SHIFTER (DS1307 runs on minimum 4.5V)
#include "DS1307.h"


#define DS1307_ADDR_SECS			(0x00)
#define DS1307_ADDR_MINUTES			(0x01)
#define DS1307_ADDR_HOURS			(0x02)
#define DS1307_ADDR_DAY				(0x03)
#define DS1307_ADDR_DATE			(0x04)
#define DS1307_ADDR_MONTH			(0x05)
#define DS1307_ADDR_YEAR			(0x06)

#define RAW_SEC						(raw_time[0])
#define RAW_MINS					(raw_time[1])
#define RAW_HOUR					(raw_time[2])

#define RAW_DAY						(raw_date[0])
#define RAW_DATE					(raw_date[1])
#define RAW_MONTH					(raw_date[2])
#define RAW_YEAR					(raw_date[3])



#define SECS_ADDDR_CH_MSK			(1 << 7)

/* HELPER FUNCTIONS START HERE */

/**
 * @brief
 * @param
 */
static uint8_t DecToBCD(uint8_t val)
{
	if(val > 99){
		return -1;
	}
	return ((val / 10) << 4) | (val % 10);
}

/**
 * @brief
 * @param
 */
static uint8_t BCDToDec(uint8_t val)
{
	return ((val >> 4) * 10) + (val & 0x0F);
}
/* HELPER FUNCTIONS END HERE */

/*************** MAIN FUNCTIONS START HERE *************************************/

/**
 * @brief Function that disables the oscillator of the module and subjects the module for new time or date values
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS137_INIT(DS1307_Handle_t* DS1307_HANDLE, uint8_t SlaveAddr)
{
	static uint8_t temp_tx_buffer[2];

	if(DS1307_HANDLE->I2C_RECEIVE(SlaveAddr, DS1307_ADDR_SECS, &temp_tx_buffer[1], 1) == 0){
		return 0;
	}

	temp_tx_buffer[0] = DS1307_ADDR_SECS;
	temp_tx_buffer[1] |= SECS_ADDDR_CH_MSK;
	if(DS1307_HANDLE->I2C_TRANSMIT(SlaveAddr, temp_tx_buffer, 2) == 0){
		return 0;
	}
	// disables the oscillator

	return 1;
}

/**
 * @brief Function used to set the current time for the module
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS1307_SET_TIME(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr)
{
	static uint8_t tx_buffer[4];

	tx_buffer[0] = DS1307_ADDR_SECS; // points to the first address for time

	// set the value for seconds register
	DS1307_HANDLE->raw_time[0] = DecToBCD(DS1307_HANDLE->DS1307_TIME_HANDLE.SECONDS) & ~(0x80); // 0x80 clears the bit 7 (Clock Halt) of 0x00 register, enables the OSC
	tx_buffer[1] = DS1307_HANDLE->raw_time[0];

	// set the value for minutes register
	DS1307_HANDLE->raw_time[1] = DecToBCD(DS1307_HANDLE->DS1307_TIME_HANDLE.MINUTES);
	tx_buffer[2] = DS1307_HANDLE->raw_time[1];

	// set the value for hour register
	DS1307_HANDLE->raw_time[2] = DecToBCD(DS1307_HANDLE->DS1307_TIME_HANDLE.HOURS);
	switch(DS1307_HANDLE->DS1307_TIME_HANDLE.HOUR_FORMAT) {
		case DS1307_24H_FORMAT:
			DS1307_HANDLE->raw_time[2] &= ~(0x40); // clears the BIT6 of hour register to set as 24 format
			break;

		case DS1307_12H_FORMAT_AM:
			DS1307_HANDLE->raw_time[2] |= (0x40);	// sets BIT6 and clears BIT5 for 12 HR format and AM
			DS1307_HANDLE->raw_time[2] &= ~(0x20);
			break;


		case DS1307_12H_FORMAT_PM:
			DS1307_HANDLE->raw_time[2] |= (0x40 | 0x20); // sets both BIT6 and BIT5 for 12 hr format and PM
			break;
		}

	return DS1307_HANDLE->I2C_TRANSMIT(Slave_Addr, tx_buffer, sizeof(tx_buffer)/sizeof(tx_buffer[0]));
}


/**
 * @brief Function used to set the current time for the module
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS1307_SET_TIME_IT(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr){
	static uint8_t tx_buff[4];

	tx_buff[0] = DS1307_ADDR_SECS;

	// set the value for seconds register
	DS1307_HANDLE->raw_time[0] = DecToBCD(DS1307_HANDLE->DS1307_TIME_HANDLE.SECONDS) & ~(0x80);
	tx_buff[1] = DS1307_HANDLE->raw_time[0];

	// set the value for minutes register
	DS1307_HANDLE->raw_time[1] = DecToBCD(DS1307_HANDLE->DS1307_TIME_HANDLE.MINUTES);
	tx_buff[2] = DS1307_HANDLE->raw_time[1];

	// set the value for hours register
	DS1307_HANDLE->raw_time[2] = DecToBCD(DS1307_HANDLE->DS1307_TIME_HANDLE.HOURS);
	switch(DS1307_HANDLE->DS1307_TIME_HANDLE.HOUR_FORMAT) {
		case DS1307_24H_FORMAT:
			DS1307_HANDLE->raw_time[2] &= ~(0x40); // clears the BIT6 of hour register to set as 24 format
			break;

		case DS1307_12H_FORMAT_AM:
			DS1307_HANDLE->raw_time[2] |= (0x40);	// sets BIT6 and clears BIT5 for 12 HR format and AM
			DS1307_HANDLE->raw_time[2] &= ~(0x20);
			break;

		case DS1307_12H_FORMAT_PM:
			DS1307_HANDLE->raw_time[2] |= (0x40 | 0x20); // sets both BIT6 and BIT5 for 12 hr format and PM
			break;
	}
	tx_buff[3] = DS1307_HANDLE->raw_time[2];

	if((DS1307_HANDLE->I2C_TRANSMIT(Slave_Addr, tx_buff, sizeof(tx_buff)/sizeof(tx_buff[0])) == DS1307_SUCCESS)){
		DS1307_HANDLE->DS1307_IT_STATUS = DS1307_IT_SET_TIME_BUSY;
		return DS1307_SUCCESS;
	}
	else {
		return DS1307_ERROR;
	}

}

/**
 * @brief Function used to set the current date for the RTC module
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS1307_SET_DATE(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr)
{
	static uint8_t tx_buffer[5];
	tx_buffer[0] = DS1307_ADDR_DAY;

	// set day of the week
	DS1307_HANDLE->raw_date[0] = DecToBCD(DS1307_HANDLE->DS1307_DATE_HANDLE.DAY_OF_THE_WEEK & 0x07);
	tx_buffer[1] = DS1307_HANDLE->raw_date[0];

	// set the date
	DS1307_HANDLE->raw_date[1] = DecToBCD(DS1307_HANDLE->DS1307_DATE_HANDLE.DAY & 0x3F);
	tx_buffer[2] = DS1307_HANDLE->raw_date[1];

	DS1307_HANDLE->raw_date[2] = DecToBCD(DS1307_HANDLE->DS1307_DATE_HANDLE.MONTH & 0x1F);
	tx_buffer[3] = DS1307_HANDLE->raw_date[2];

	DS1307_HANDLE->raw_date[3] = DecToBCD(DS1307_HANDLE->DS1307_DATE_HANDLE.YEAR);
	tx_buffer[4] = DS1307_HANDLE->raw_date[3];

	return DS1307_HANDLE->I2C_TRANSMIT(Slave_Addr, tx_buffer, sizeof(tx_buffer)/sizeof(tx_buffer[0]));
}

/**
 * @brief Interrupt function used to set the current date for the RTC module
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS1307_SET_DATE_IT(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr)
{
	static uint8_t tx_buffer[5];
	tx_buffer[0] = DS1307_ADDR_DAY;

	// set day of the week
	DS1307_HANDLE->raw_date[0] = DecToBCD(DS1307_HANDLE->DS1307_DATE_HANDLE.DAY_OF_THE_WEEK & 0x07);
	tx_buffer[1] = DS1307_HANDLE->raw_date[0];

	// set the date
	DS1307_HANDLE->raw_date[1] = DecToBCD(DS1307_HANDLE->DS1307_DATE_HANDLE.DAY & 0x3F);
	tx_buffer[2] = DS1307_HANDLE->raw_date[1];

	DS1307_HANDLE->raw_date[2] = DecToBCD(DS1307_HANDLE->DS1307_DATE_HANDLE.MONTH & 0x1F);
	tx_buffer[3] = DS1307_HANDLE->raw_date[2];

	DS1307_HANDLE->raw_date[3] = DecToBCD(DS1307_HANDLE->DS1307_DATE_HANDLE.YEAR);
	tx_buffer[4] = DS1307_HANDLE->raw_date[3];

	if(DS1307_HANDLE->I2C_TRANSMIT(Slave_Addr, tx_buffer, sizeof(tx_buffer)/sizeof(tx_buffer[0])) == DS1307_SUCCESS){
		DS1307_HANDLE->DS1307_IT_STATUS = DS1307_IT_SET_DATE_BUSY;
		return DS1307_SUCCESS;
	}
	else {
		return DS1307_ERROR;
	}
}

/**
 * @brief Polling Function in retrieving the current time from the module
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS1307_GET_TIME(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr)
{
	uint8_t tempreg = 0;
	//receive hours value
	if(DS1307_HANDLE->I2C_RECEIVE(Slave_Addr, DS1307_ADDR_HOURS, &tempreg, 1) == DS1307_ERROR){
		return DS1307_ERROR; // timeout error
	}
	if(tempreg & (0x01 << 6)){
		if(tempreg & (1 << 5)){
			DS1307_HANDLE->DS1307_TIME_HANDLE.HOUR_FORMAT = DS1307_12H_FORMAT_PM;
		}
		else {
			DS1307_HANDLE->DS1307_TIME_HANDLE.HOUR_FORMAT = DS1307_12H_FORMAT_AM;
		}
		DS1307_HANDLE->DS1307_TIME_HANDLE.HOURS = BCDToDec(tempreg & ~(0x03 << 5)); // clear the mask for BIT5 and BIT6, BIT4~BIT0 are 12 hour values
	}
	else {
		DS1307_HANDLE->DS1307_TIME_HANDLE.HOUR_FORMAT = DS1307_24H_FORMAT;
		DS1307_HANDLE->DS1307_TIME_HANDLE.HOURS = BCDToDec(tempreg & ~(0x01 << 6)); // clear the mask for BIT6, BIT5~BIT0 are 24 hour values
	}
	// receive minutes value
	if(DS1307_HANDLE->I2C_RECEIVE(Slave_Addr, DS1307_ADDR_MINUTES, &tempreg, 1) == DS1307_ERROR){
		return DS1307_ERROR; // timeout error
	}
	DS1307_HANDLE->DS1307_TIME_HANDLE.MINUTES = BCDToDec(tempreg);

	// receive seconds value
	if(DS1307_HANDLE->I2C_RECEIVE(Slave_Addr, DS1307_ADDR_SECS, &tempreg, 1) == DS1307_ERROR){
		return DS1307_ERROR; // timeout error
	}
	DS1307_HANDLE->DS1307_TIME_HANDLE.SECONDS = BCDToDec(tempreg & 0x7F);

	return DS1307_SUCCESS;
}

/**
 * @brief Interrupt mode Function in retrieving the current time from the module
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS1307_GET_TIME_IT(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr)
{
	if( DS1307_HANDLE->I2C_RECEIVE(Slave_Addr,
			DS1307_ADDR_SECS,									/* Set DS1307 pointer to secs addr and automatically increments from there */
			DS1307_HANDLE->raw_time,
			raw_time_len) == 1){
		DS1307_HANDLE->DS1307_IT_STATUS = DS1307_IT_GET_TIME_BUSY;
		return DS1307_SUCCESS;
		 /* Your Rx-complete ISR should copy the last byte, then do:      */
		 /* DS1307_HANDLE->DS1307_IT_STATUS = DS1307_IT_FREE;             */
	}
	else {
		return DS1307_ERROR;
	}
}

/**
 * @brief Function that handles the processing once the DS1307_GET_TIME_IT completes receiving
 *        the burst raw 3 byte data of time
 * @param DS1307_HANDLE : Handle of ds1307 containing the raw data and usable data for time and date
 */
void DS1307_CONVERT_RAW_TIME(DS1307_TIME_t* DS1307_TIME_HANDLE, const uint8_t* raw_time)
{
	/* Make sure that ISR has completely received the raw bytes for time */
	/* Process raw hour data */
	if(RAW_HOUR & (0x01 << 6)){
		if(RAW_HOUR & (1 << 5)){
			DS1307_TIME_HANDLE->HOUR_FORMAT = DS1307_12H_FORMAT_PM;
		}
		else {
			DS1307_TIME_HANDLE->HOUR_FORMAT = DS1307_12H_FORMAT_AM;
		}
		DS1307_TIME_HANDLE->HOURS = BCDToDec(RAW_HOUR & ~(0x03 << 5)); // clear the mask for BIT5 and BIT6, BIT4~BIT0 are 12 hour values
	}
	else {
		DS1307_TIME_HANDLE->HOUR_FORMAT = DS1307_24H_FORMAT;
		DS1307_TIME_HANDLE->HOURS = BCDToDec(RAW_HOUR & ~(0x01 << 6)); // clear the mask for BIT6, BIT5~BIT0 are 24 hour values
	}

	/*　receive minutes value　*/
	DS1307_TIME_HANDLE->MINUTES = BCDToDec(RAW_MINS);

	/*　receive seconds value　*/
	DS1307_TIME_HANDLE->SECONDS = BCDToDec(RAW_SEC & 0x7F);
}

/**
 * @brief P Function in retrieving the current date from the RTC module
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS1307_GET_DATE(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr)
{
	uint8_t temp_reg;


	// receive day of the week
	if(DS1307_HANDLE->I2C_RECEIVE(Slave_Addr, DS1307_ADDR_DAY, &temp_reg, 1) == DS1307_ERROR){
		return DS1307_ERROR;
	}

	DS1307_HANDLE->DS1307_DATE_HANDLE.DAY_OF_THE_WEEK = BCDToDec(temp_reg & (0x07)); // clear the mask for BIT3 ~ BIT7

	// receive date
	if(DS1307_HANDLE->I2C_RECEIVE(Slave_Addr, DS1307_ADDR_DATE, &temp_reg, 1) == DS1307_ERROR){
		return DS1307_ERROR;
	}
	DS1307_HANDLE->DS1307_DATE_HANDLE.DAY = BCDToDec(temp_reg & (0x3F)); // clear the mask for BIT6 ~ BIT7

	// receive month
	if(DS1307_HANDLE->I2C_RECEIVE(Slave_Addr, DS1307_ADDR_MONTH, &temp_reg, 1) == DS1307_ERROR){
		return DS1307_ERROR;
	}
	DS1307_HANDLE->DS1307_DATE_HANDLE.MONTH = BCDToDec(temp_reg & (0x1F)); // clear the mask for BIT5 ~ BIT7

	// receive year
	if(DS1307_HANDLE->I2C_RECEIVE(Slave_Addr, DS1307_ADDR_MONTH, &temp_reg, 1) == DS1307_ERROR){
		return DS1307_ERROR;
	}
	DS1307_HANDLE->DS1307_DATE_HANDLE.YEAR = BCDToDec(temp_reg);

	return DS1307_SUCCESS;
}


/**
 * @brief Interrupt Function in retrieving the current date from the RTC module
 * @param DS1307_HANDLE : pointer to struct handler of DS1307
 * @param Slave_Addr : Slave address of the ds1307 module
 */
uint8_t DS1307_GET_DATE_IT(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr)
{
	if(DS1307_HANDLE->I2C_RECEIVE(Slave_Addr,
			DS1307_ADDR_DAY,
			DS1307_HANDLE->raw_date,
			raw_date_len
			) == 1){

		 /* Your Rx-complete ISR should copy the last byte, then do:      */
		 /* DS1307_HANDLE->DS1307_IT_STATUS = DS1307_IT_FREE;             */
		DS1307_HANDLE->DS1307_IT_STATUS = DS1307_IT_GET_DATE_BUSY;
		return DS1307_SUCCESS;
	}
	else {
		DS1307_HANDLE->DS1307_IT_STATUS = DS1307_IT_FREE;
		return DS1307_ERROR;
	}

}

void DS1307_CONVERT_RAW_DATE(DS1307_DATE_t* DS1307_DATE_HANDLE, const uint8_t* raw_date)
{
	// receive day of the week
	DS1307_DATE_HANDLE->DAY_OF_THE_WEEK = BCDToDec(RAW_DAY & (0x07)); // clear the mask for BIT3 ~ BIT7

	// receive date
	DS1307_DATE_HANDLE->DAY = BCDToDec(RAW_DATE & (0x3F)); // clear the mask for BIT6 ~ BIT7

	// receive month
	DS1307_DATE_HANDLE->MONTH = BCDToDec(RAW_MONTH & (0x1F)); // clear the mask for BIT5 ~ BIT7

	// receive year
	DS1307_DATE_HANDLE->YEAR = BCDToDec(RAW_YEAR);
}

/**
 * @brief Function to convert Date variables to string
 * @param DS1307_DATE: Handle that stores the date related values
 * @param date_holder: to hold the string equivalent of current date as a whole
 * @param date_holder_size : date_holder size in bytes
 */
void DateToString(const DS1307_DATE_t* DS1307_DATE, char *date_holder, uint32_t date_holder_size){
	char day[7][4] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
	if(DS1307_DATE->DAY_OF_THE_WEEK <= SATURDAY){
		snprintf(date_holder, date_holder_size,
				"%02d/%02d/20%02d %s",
				DS1307_DATE->MONTH,
				DS1307_DATE->DAY,
				DS1307_DATE->YEAR,
				day[DS1307_DATE->DAY_OF_THE_WEEK]);
	}
	else {
		snprintf(date_holder,
				date_holder_size,
				"%02d/%02d/20%02d ???",
				DS1307_DATE->MONTH,
				DS1307_DATE->DAY,
				DS1307_DATE->YEAR);
	}
}

/**
 * @brief Function to convert Time variables to string
 * @param DS1307_TIME: Handle that stores the Time values
 * @param time_holder: output array that will hold the converted string of current time
 * @param time_holder_size : time_holder parameter size in bytes
 */
void TimeToString(const DS1307_TIME_t* DS1307_TIME, char *time_holder, uint32_t time_holder_size){

	if(DS1307_TIME->HOUR_FORMAT == DS1307_24H_FORMAT){
		snprintf(time_holder, time_holder_size, "%02d:%02d:%02d MT", DS1307_TIME->HOURS, DS1307_TIME->MINUTES, DS1307_TIME->SECONDS);

	}
	else if(DS1307_TIME->HOUR_FORMAT == DS1307_12H_FORMAT_AM){
		snprintf(time_holder, time_holder_size, "%02d:%02d:%02d AM", DS1307_TIME->HOURS, DS1307_TIME->MINUTES, DS1307_TIME->SECONDS);

	}
	else if(DS1307_TIME->HOUR_FORMAT == DS1307_12H_FORMAT_PM){
		snprintf(time_holder, time_holder_size, "%02d:%02d:%02d PM", DS1307_TIME->HOURS, DS1307_TIME->MINUTES, DS1307_TIME->SECONDS);

	}
	else {
		snprintf(time_holder, time_holder_size, "--:--:-- --");
	}
}

/**
 * @brief Function used to convert Day variable to string
 * @param DS1307_DATE: Handle that stores the date related values
 * @param date_holder: to hold the string equivalent of current date as a whole
 * @param day_holder_size : day_holder parameter size in bytes
 *
 */
void DayToString(const DS1307_DATE_t* DS1307_DATE, char *day_holder, uint32_t day_holder_size){
	char day[7][4] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
	if(DS1307_DATE->DAY <= SATURDAY){
		snprintf(day_holder, day_holder_size, "%s", day[DS1307_DATE->DAY]);
	}
	else {
		snprintf(day_holder, day_holder_size, "???");
	}
}

/*************** MAIN FUNCTIONS END HERE *************************************/

