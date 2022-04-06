# SMART_Home

Introducing the SMART Home Real-Time IOS Device


SMART Home Real-Time IOT Device is a sensor platform intended for smart home applications or home 
monitoring. The main data processing unit is a STM32f411 Nucleo board, which is assigned to communicate 
with the sensors and process its raw data.

The main sensor component is the BME280 Environmental sensor, which is capable of measuring temperature, 
humidity and pressure. The sensor can communicate with the microcontroller using I2C communication 
protocol.

The device is capable of Real-Time environmental monitoring using FreeRTOS. The device’s main features are 
temperature, humidity and pressure measuring. The raw data coming from the sensor is converted to SI metrics.
After connecting the device with USB, the user can see the measured data sent with USART communication 
protocol.

Main components:

• ARM Cortex-M4 microprocessor on the STM32f411 Nucleo board.
• Bosch BME280 Environmental sensor
• FreeRTOS for real-time operation software
