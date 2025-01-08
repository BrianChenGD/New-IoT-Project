#pragma once
#define USE_NIMBLE

#define USE_RGB_LED
#define pRGB_LED 17

#define USE_GPS
#define pGPS_TX 21
#define pGPS_RX 20
#define GPS_BAUD 9600
#define pGPS_VCTRL 37

#define pDIO1 5
#define pDIO2 35
#define pDIO3 34
#define pPMC 7

#pragma once
#define USE_AIR_PRESSURE
#define USE_DSP310_I2C
#define USE_2_WIRE_SENSOR_BUS
#define DSP310_I2C_ADDRESS 0x77 // 假设使用 DSP310 传感器的 I2C 地址
#define pDSP310_I2C_SCL 19
#define pDSP310_I2C_SDA 18
#define pVext 4

#define USE_MAGNETOMETER
#define MQC5883_I2C_ADDRESS 0x0D // 假设使用 MQC5883 传感器的 I2C 地址
#define pMQC5883_I2C_SCL 19
#define pMQC5883_I2C_SDA 18

#define USE_MOTION
#define MPU6050_I2C_ADDRESS 0x68
#define pMPU6050_I2C_SCL 19
#define pMPU6050_I2C_SDA 18
