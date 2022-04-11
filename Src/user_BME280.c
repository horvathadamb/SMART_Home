/*
 * user_BME280.c
 *
 *  Created on: Mar 28, 2022
 *      Author: Horvath Adam
 */

#include "user_BME280.h"
#include "stdlib.h"

extern I2C_HandleTypeDef hi2c1;
extern BME280_Error_StatusTypeDef BME280_ErrorStatus;
extern uint8_t device_id_data;
int32_t temperature_fine;

// Parsed calib values for data processing
uint16_t T1,P1,H1,H3,H6;
int16_t T2,T3,P2,P3,P4,P5,P6,P7,P8,P9,H2,H4,H5;








/*
 * Configure the BME280 sensor
 * @t_sb 		sample time ms
 * @filter 		IIR filter
 * @spi3w_en 	SPI 3 wire mode
 * @osrs_t 		Temperature oversampling
 * @osrs_p 		Pressure oversampling
 * @osrs_h 		Humidity oversampling
 * @mode 		Sensor mode
 */
int bme280_config(uint8_t t_sb,uint8_t filter,uint8_t spi3w_en,
		uint8_t osrs_t,uint8_t osrs_p,uint8_t osrs_h,uint8_t mode)
{
	// Check device ID
	if(device_id_data != BME280_DEVICE_ID)
	{
		BME280_ErrorStatus = Device_ID_Error;
		return BME280_ErrorStatus;
	}

	uint8_t config_reset = 0x80;
	uint8_t config_buffer[12];
	uint8_t config_data;
	uint8_t hum_data;
	uint8_t meas_data;


	// Reset device
	bme280_I2C_write(&hi2c1, BME280_CONFIG_ADDR, config_reset);
	bme280_I2C_read(&hi2c1, BME280_CONFIG_ADDR, &config_buffer[0], 1);
	if(config_buffer[0] != config_reset)
	{
		BME280_ErrorStatus = Config_Error;
		return BME280_ErrorStatus;
	}

	// Config device
	config_data = t_sb | filter | spi3w_en;

	bme280_I2C_write(&hi2c1, BME280_CONFIG_ADDR, config_data);
	bme280_I2C_read(&hi2c1, BME280_CONFIG_ADDR, &config_buffer[1], 1);
	if(config_buffer[1] != config_data)
	{
		BME280_ErrorStatus = Config_Error;
		return BME280_ErrorStatus;
	}

	// Control Humidity
	hum_data = osrs_h;

	bme280_I2C_write(&hi2c1, BME280_CTRL_HUM_ADDR, hum_data);
	bme280_I2C_read(&hi2c1, BME280_CTRL_HUM_ADDR, &config_buffer[2], 1);
	if(config_buffer[2] != hum_data)
	{
		BME280_ErrorStatus = CTRL_Hum_Error;
		return BME280_ErrorStatus;
	}

	// Control Meas (Temp,Pressure)
	meas_data = osrs_t | osrs_p | mode;

	bme280_I2C_write(&hi2c1, BME280_CTRL_MEAS_ADDR, meas_data);
	bme280_I2C_read(&hi2c1, BME280_CTRL_MEAS_ADDR, &config_buffer[3], 1);
	if(config_buffer[3] != meas_data)
	{
		BME280_ErrorStatus = CTRL_Hum_Error;
		return BME280_ErrorStatus;
	}

	return 0;
}


/*
 * Parse 8bit calibration register values into 16 bit
 * Used later to calculate SI values from raw data
 * @calib_data 	Calibrate register values
 */
void parse_compensate(uint8_t * calib_data)
{
	// Check device ID
	if(device_id_data != BME280_DEVICE_ID)
	{
		BME280_ErrorStatus = Device_ID_Error;
		return BME280_ErrorStatus;
	}

	// Temperature compensation 0x88-0x8D
	T1 = (calib_data[1] << 8) | calib_data[0];
	T2 = (calib_data[3] << 8) | calib_data[2];
	T3 = (calib_data[5] << 8) | calib_data[4];

	// Pressure compensation 0x8E-0x9F
	P1 = (calib_data[7] << 8) | calib_data[6];
	P2 = (calib_data[9] << 8) | calib_data[8];
	P3 = (calib_data[11] << 8) | calib_data[10];
	P4 = (calib_data[13] << 8) | calib_data[12];
	P5 = (calib_data[15] << 8) | calib_data[14];
	P6 = (calib_data[17] << 8) | calib_data[16];
	P7 = (calib_data[19] << 8) | calib_data[18];
	P8 = (calib_data[21] << 8) | calib_data[20];
	P9 = (calib_data[23] << 8) | calib_data[22];

	// Humidity compensation 0xA1,0xE1-0xE7
	H1 = calib_data[24];
	H2 = (calib_data[26] << 8) | calib_data[25];
	H3 = calib_data[27];
	H4 = (calib_data[28] << 4) | (calib_data[29] & 0x0F);
	H5 = (calib_data[30] << 4) | calib_data[29] >>4;
	H6 = calib_data[31];
}

/*
 * Measure temperature in °C
 * @sensor_raw_data The value stored in sensor temperature registers
 */
int32_t bme280_measure_temperature_int32(uint8_t* sensor_raw_data, int32_t* pData)
{
	// Check for nullpointer
	if(pData == NULL)
	{
		BME280_ErrorStatus = NullPointer_Error;
		return BME280_ErrorStatus;
	}

	// Check device ID
	if(device_id_data != BME280_DEVICE_ID)
	{
		BME280_ErrorStatus = Device_ID_Error;
		return BME280_ErrorStatus;
	}

	int32_t var1,var2,temperature;
	int32_t temp_raw;

	// Create raw temperature value from 20bit data
    temp_raw = ((uint32_t)sensor_raw_data[3] << 12) | ((uint32_t)sensor_raw_data[4] << 4) | ((uint32_t)sensor_raw_data[5] >> 4);

    // Calculate °C value *100
    var1 = ((((temp_raw>>3) - ((int32_t)T1<<1))) * ((int32_t)T2)) >> 11;
	var2 = (((((temp_raw>>4) - ((int32_t)T1)) * ((temp_raw>>4) - ((int32_t)T1)))>> 12) *((int32_t)T3)) >> 14;
	temperature_fine = var1 + var2;
	temperature = (temperature_fine * 5 + 128) >> 8;
	*pData = temperature;
	return 0;
}

/* Measure Pressure in hPa
* @sensor_raw_data The value stored in sensor pressure registers
*/
uint32_t bme280_measure_pressure_int32(uint8_t* sensor_raw_data, uint32_t * pData)
{
	// Check for nullpointer
	if(pData == NULL)
		{
			BME280_ErrorStatus = NullPointer_Error;
			return BME280_ErrorStatus;
		}

	// Check device ID
	if(device_id_data != BME280_DEVICE_ID)
	{
		BME280_ErrorStatus = Device_ID_Error;
		return BME280_ErrorStatus;
	}

	int32_t var1,var2;
	int32_t pressure_raw, pressure;

	// Create raw pressure value from 20bit data
    pressure_raw = ((uint32_t)sensor_raw_data[0] << 12) | ((uint32_t)sensor_raw_data[1] << 4) | ((uint32_t)sensor_raw_data[2] >> 4);

    // Calculate Pa
    var1 = (((int32_t)temperature_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)P6);
	var2 = var2 + ((var1*((int32_t)P5))<<1);
 	var2 = (var2>>2)+(((int32_t)P4)<<16);
 	var1 = (((P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)P2) *var1)>>1))>>18;
 	var1 =((((32768+var1))*((int32_t)P1))>>15);
 	if (var1 == 0)
 	{
 		return 0; // avoid exception caused by division by zero
    }
 	pressure = (((uint32_t)(((int32_t)1048576)-pressure_raw)-(var2>>12)))*3125;
    if (pressure < 0x80000000)
    {
    	pressure = (pressure << 1) / ((uint32_t)var1);
    }
    else
    {
    	pressure = (pressure / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)P9) * ((int32_t)(((pressure>>3) * (pressure>>3))>>13)))>>12;
    var2 = (((int32_t)(pressure>>2)) * ((int32_t)P8))>>13;
    pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + P7) >> 4));

    *pData = pressure;

    return 0;
}


/* Measure relative humidity %
* @sensor_raw_data The value stored in sensor humidity registers
*/
uint32_t bme280_measure_humidity_int32(uint8_t* sensor_raw_data,uint32_t * pData)
{
	// Check for nullpointer
	if(pData == NULL)
		{
			BME280_ErrorStatus = NullPointer_Error;
			return BME280_ErrorStatus;
		}

	// Check device ID
	if(device_id_data != BME280_DEVICE_ID)
	{
		BME280_ErrorStatus = Device_ID_Error;
		return BME280_ErrorStatus;
	}

	int32_t v_x1_u32r;
	int32_t humidity_raw;

	// Create raw pressure value from 20bit data
	humidity_raw = ((uint32_t)sensor_raw_data[6] << 8) | (uint32_t)sensor_raw_data[7];

	// Calculate realtive % humidity
	v_x1_u32r = (temperature_fine - ((int32_t)76800));
	v_x1_u32r = (((((humidity_raw << 14) - (((int32_t)H4) << 20) - (((int32_t)H5) *\
			v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *\
					((int32_t)H6)) >> 10) * (((v_x1_u32r * ((int32_t)H3)) >> 11) +\
							((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)H2) +\
					8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *\
			((int32_t)H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	*pData = (uint32_t)(v_x1_u32r>>12);
	return 0;
}

/*
 * Read sensor memory using I2C
 * @*hi2c 		pointer to I2C handle
 * @MemAddress 	address of register to read
 * @*pdata 		pointer to save data
 * @size 		number of bytes read
 */
int bme280_I2C_read(I2C_HandleTypeDef *hi2c,
		uint16_t MemAddress,
		uint8_t *pData,
		uint16_t Size)
{
	HAL_StatusTypeDef I2C_Status;
	I2C_Status = HAL_I2C_Mem_Read(&hi2c1, BME280_DEV_I2C_ADDR, (uint8_t*)MemAddress, 1, pData, Size, 100);
	// Check if I2C communication was successful
	if(I2C_Status != HAL_OK)
	{
		BME280_ErrorStatus = I2C_Error;
		return BME280_ErrorStatus; // If not return error
	}

	return 0;
}

/*
 * Write sensor memory using I2C
 * @*hi2c 		pointer to I2C handle
 * @MemAddress 	address of register to write
 * @*pdata 		pointer to write data
 */
int bme280_I2C_write(I2C_HandleTypeDef *hi2c,
		uint16_t MemAddress,
		uint8_t *pData)
{
	HAL_StatusTypeDef I2C_Status;
	I2C_Status = HAL_I2C_Mem_Write(hi2c, BME280_DEV_I2C_ADDR, MemAddress, 1, &pData, 1, 100);
	// Check if I2C communication was successful
	if(I2C_Status != HAL_OK)
		{
		BME280_ErrorStatus = I2C_Error;
		return BME280_ErrorStatus; // If not return error
		}
	return 0;
}

