/*
 * user_BME280.h
 *
 *  Created on: Mar 28, 2022
 *      Author: Horvath Adam
 */

#ifndef INC_USER_BME280_H_
#define INC_USER_BME280_H_

#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stdint.h"

// Sensor Errors
typedef enum
{
	No_Error = 				0x00,
	Device_ID_Error = 		0x04,
	NullPointer_Error = 	0x08,
	Config_Error = 			0x16,
	CTRL_Hum_Error = 		0x20,
	CTRL_Meas_Error = 		0x22,
	I2C_Error = 			0x32

} Error_StatusTypeDef;

/*---------------Register Address---------------------*/
// Device
#define BME280_DEV_I2C_ADDR 			(0x77 << 1)
#define BME280_ID_ADDR					0xD0
#define BME280_DEVICE_ID				0x60u

// Config
#define BME280_CONFIG_ADDR 				0xF5
#define BME280_CTRL_MEAS_ADDR 			0xF4
#define BME280_CTRL_HUM_ADDR 			0xF2

// Temperature
#define BME280_TEMP_MSB_ADDR 			0xFA
#define BME280_TEMP_LSB_ADDR 			0xFB
#define BME280_TEMP_XLSB_ADDR 			0xFC

// Pressure
#define BME280_PRESS_MSB_ADDR 			0xF7
#define BME280_PRESS_LSB_ADDR 			0xF8
#define BME280_PRESS_XLSB_ADDR 			0xF9

// Humidity
#define BME280_HUM_MSB_ADDR 			0xFD
#define BME280_HUM_LSB_ADDR 			0xFE

// Calibrate
#define BME280_CALIB_00_ADDR 			0x88
#define BME280_CALIB_26_ADDR 			0xE1


/*------------------Config-----------------------*/

//Spi 3 wire enable [2:0]
#define CONFIG_SPI3W_DISABLE				0x00
#define CONFIG_SPI3W_ENABLE					0x01

// Filter [4:2]
#define CONFIG_IIR_FILTER_OFF				(0x00<<2) // Changes temperature resolution from 20bit to 16bit!
#define CONFIG_IIR_FILTER_2					(0x01<<2)
#define CONFIG_IIR_FILTER_4					(0x02<<2)
#define CONFIG_IIR_FILTER_8					(0x03<<2)
#define CONFIG_IIR_FILTER_16				(0x04<<2)

// Sample time ms
#define CONFIG_T_SB_05						(0x00<<5)
#define CONFIG_T_SB_62_5					(0x01<<5)
#define CONFIG_T_SB_125						(0x02<<5)
#define CONFIG_T_SB_250						(0x03<<5)
#define CONFIG_T_SB_500						(0x04<<5)
#define CONFIG_T_SB_1000					(0x05<<5)
#define CONFIG_T_SB_10						(0x06<<5)
#define CONFIG_T_SB_20						(0x07<<5)

// Mode [2:0]
#define CTRL_MEAS_MODE_SLEEP				0x00
#define CTRL_MEAS_MODE_FORCED				0x01
#define CTRL_MEAS_MODE_NORMAL				0x03

// Oversampling pressure [4:2]
#define CTRL_MEAS_OSRS_P_SKIPPED			(0x00<<2)
#define CTRL_MEAS_OSRS_P_1					(0x01<<2)
#define CTRL_MEAS_OSRS_P_2					(0x02<<2)
#define CTRL_MEAS_OSRS_P_4					(0x03<<2)
#define CTRL_MEAS_OSRS_P_8					(0x04<<2)
#define CTRL_MEAS_OSRS_P_16					(0x05<<2)

// Oversampling temperature [7:5]
#define CTRL_MEAS_OSRS_T_SKIPPED			(0x00<<5)
#define CTRL_MEAS_OSRS_T_1					(0x01<<5)
#define CTRL_MEAS_OSRS_T_2					(0x02<<5)
#define CTRL_MEAS_OSRS_T_4					(0x03<<5)
#define CTRL_MEAS_OSRS_T_8					(0x04<<5)
#define CTRL_MEAS_OSRS_T_16					(0x05<<5)

// Oversampling humidity [2:0]
#define CTRL_HUM_OSRS_H_SKIPPED				0x00
#define CTRL_HUM_OSRS_H_1					0x01
#define CTRL_HUM_OSRS_H_2					0x02
#define CTRL_HUM_OSRS_H_4					0x03
#define CTRL_HUM_OSRS_H_8					0x04
#define CTRL_HUM_OSRS_H_16					0x05


/*--------------------Function prototype-----------------------*/


// Measure temperature and calculate °C value from raw data
int32_t bme280_measure_temperature_int32(uint8_t* sensor_raw_data,  int32_t* pData);

 // Measure pressure and calculate hPa value from raw data
uint32_t bme280_measure_pressure_int32(uint8_t* sensor_raw_data,uint32_t * pData);

// Measure humidity and calculate relateive humidity %
uint32_t bme280_measure_humidity_int32(uint8_t* sensor_raw_data, uint32_t * pData);

// Parse calibration data for constants
void parse_compensate(uint8_t * calib_data);

// Write to sensor with I2C
int bme280_I2C_write(I2C_HandleTypeDef *hi2c,
		uint16_t MemAddress,
		uint8_t *pData);

// Read to sensor with I2C
int bme280_I2C_read(I2C_HandleTypeDef *hi2c,
		uint16_t MemAddress,
		uint8_t *pData,
		uint16_t Size);

// Configure the sensor
int bme280_config(uint8_t t_sb,uint8_t filter,uint8_t spi3w_en,
		uint8_t osrs_t,uint8_t osrs_p,uint8_t osrs_h,uint8_t mode);

#endif /* INC_USER_BME280_H_ */
