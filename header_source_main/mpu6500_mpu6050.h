/*
 * mpu6500_mpu6050.h
 *
 *  Created on: Aug 11, 2025
 *      Author: Enes
 */

#ifndef INC_MPU6500_MPU6050_H_
#define INC_MPU6500_MPU6050_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "stdbool.h"

#define MPU6xx0_WHO_AM_I_ADDR 0x75
#define MPU6050_WHO_AM_I      0x68
#define MPU6500_WHO_AM_I      0x70
#define MPU6xx0_I2C_ADDR_0   (0x68 << 1)
#define MPU6xx0_I2C_ADDR_1   (0x69 << 1)
#define MPU6xx0_SAMPLRT_ADDR       0x19
#define MPU6xx0_CONFIG_ADDR        0x1A
#define MPU6xx0_GYRO_CONFIG_ADDR   0x1B
#define MPU6xx0_ACCEL_CONFIG_ADDR  0x1C
#define MPU6xx0_PWR_MGMT_1          0x6B
#define MPU6xx0_PWR_MGMT_2          0x6C
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_GYRO_XOUT_H 	0x43
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_INT_PIN_ADDR        0x37
#define MPU6050_INT_ENA_ADDR        0x38
#define MPU6050_INT_STATUS_ADDR     0x3A
#define MPU6050_WAKE_ON_MOTION_ADDR	0x1F
#define MPU6050_FIFO_ENABLE_ADDR	0x23
#define MPU6050_FIFO_READ_ADDR		0x74
#define MPU6050_USER_CONTROL_ADDR	0x6A

typedef enum {
	GYRO_250_DPS = 0, GYRO_500_DPS = 1, GYRO_1000_DPS = 2, GYRO_2000_DPS = 3,
} MPU6xx0_Gyro_Config;

typedef enum {
	GYRO_250_TEMP_4000_BANDWIDTH = 0,
	GYRO_184_TEMP_188_BANDWIDTH,
	GYRO_92_TEMP_98_BANDWIDTH,
	GYRO_41_TEMP_42_BANDWIDTH,
	GYRO_20_TEMP_20_BANDWIDTH,
	GYRO_10_TEMP_10_BANDWIDTH,
	GYRO_5_TEMP_5_BANDWIDTH,
	GYRO_3600_TEMP_4000_BANDWIDTH,
} MPU6xx0_Gyro_Config2;

typedef enum {
	ACCEL_2G = 0, ACCEL_4G = 1, ACCEL_8G = 2, ACCEL_16G = 3,
} MPU6xx0_Accel_Config;

typedef enum {
	ACCEL_460_BANDWIDTH = 0,
	ACCEL_184_BANDWIDTH,
	ACCEL_92_BANDWIDTH,
	ACCEL_41_BANDWIDTH,
	ACCEL_20_BANDWIDTH,
	ACCEL_10_BANDWIDTH,
	ACCEL_5_BANDWIDTH,
} MPU6xx0_Accel_Config2;

typedef enum {
	MPU6xx0_OK = 0,
	DEVICE_NOT_FOUND,
	WHO_IS_THIS_DEVICE,
	REGISTER_READ_UNAVAILABLE,
	REGISTER_WRITE_UNAVAILABLE,
	UNINITED_DEVICE,
} MPU6xx0_Status;

typedef struct {
	uint8_t Reserved :1;
	uint8_t Fifo_Mode :1;
	uint8_t Ext_Sync_Set :3;
	uint8_t Dlpf_Cfg :3;
} MPU6xx0_Config_t;

typedef struct{
	uint8_t Actl: 1;
	uint8_t Open: 1;
	uint8_t Latch_Int_En: 1;
	uint8_t Int_Anyrd_2clear: 1;
	uint8_t Actl_Fsync: 1;
	uint8_t Fsync_Int_Mode_En: 1;
	uint8_t Bypass_En: 1;
	uint8_t Reserved: 1;
}MPU6xx0_Interrupt_Pin;

typedef struct{
	uint8_t Reserved: 4;
	uint8_t Wom_En: 1;
	uint8_t Fifo_OverFlow_En: 1;
	uint8_t Fsync_Int_En: 1;
	uint8_t RAW_RDY_EN: 1;
}MPU6xx0_Interrupt_Enable;

typedef struct{
	uint8_t Reserved: 3;
	uint8_t Wom_INT: 1;
	uint8_t Fifo_OverFlow_INT: 1;
	uint8_t FSYNC_INT: 1;
	uint8_t RAW_DATA_RDY_INT: 1;
	uint8_t DMP_INT: 1;
}MPU6xx0_Interrupt_Status;

typedef struct{
	uint8_t TEMP_OUT: 1;
	uint8_t GYRO_XOUT: 1;
	uint8_t GYRO_YOUT: 1;
	uint8_t GYRO_ZOUT: 1;
	uint8_t ACCEL: 1;
	uint8_t SLV_2: 1;
	uint8_t SLV_1: 1;
	uint8_t SLV_0: 1;
}MPU6xx0_FIFO_Enable;

typedef struct {
	uint8_t Device_Reset :1;
	uint8_t Sleep :1;
	uint8_t Cycle :1;
	uint8_t Gyro_Standby :1;
	uint8_t Temp_Dis :1;
	uint8_t ClkSel :3;
} MPU6xx0_Power_Management_1_t;

typedef struct {
	uint8_t Lp_Wake_Ctrl :2;
	uint8_t Disable_xa :1;
	uint8_t Disable_ya :1;
	uint8_t Disable_za :1;
	uint8_t Disable_xg :1;
	uint8_t Disable_yg :1;
	uint8_t Disable_zg :1;
} MPU6xx0_Power_Management_2_t;

typedef struct {
	I2C_HandleTypeDef *i2c_handle;
	uint8_t i2c_add;
	uint8_t who_am_i;

	int8_t accel_xout_h;
	int8_t accel_xout_l;
	int8_t accel_yout_h;
	int8_t accel_yout_l;
	int8_t accel_zout_h;
	int8_t accel_zout_l;
	int8_t temp_out_h;
	int8_t temp_out_l;
	int8_t gyro_xout_h;
	int8_t gyro_xout_l;
	int8_t gyro_yout_h;
	int8_t gyro_yout_l;
	int8_t gyro_zout_h;
	int8_t gyro_zout_l;

	// Buffer for raw data from sensor
	uint8_t raw_data[14]; // 6 bytes accel, 2 bytes temp, 6 bytes gyro

	// Final, calculated values in physical units
	int16_t accel_G[3];     // X, Y, Z in G's
	int16_t gyro_dps[3];    // X, Y, Z in degrees per second
	float temperature_C;  // Temperature in Celsius

	float accelIng[3];
	float gyroIns[3];
} MPU6xx0_t;

typedef struct {
	uint8_t Sample_Rate;
	MPU6xx0_Config_t Config;

	uint8_t Fchoice_B;
	MPU6xx0_Gyro_Config Gyro_FS_Sel;
	MPU6xx0_Gyro_Config2 Dlpf_Cfg;

	MPU6xx0_Accel_Config Accel_FS_Sel;
	uint8_t Accel_Fchoice_B;
	MPU6xx0_Accel_Config2 A_Dlpf_Cfg;

	MPU6xx0_Power_Management_1_t Power_Manage1;
	MPU6xx0_Power_Management_2_t Power_Manage2;

	MPU6xx0_Interrupt_Enable Int_Enable;
	MPU6xx0_Interrupt_Pin	 Int_pin;
	MPU6xx0_Interrupt_Status Int_Stats;

	MPU6xx0_FIFO_Enable FIFO_Enable;

} MPU6xx0_Configuration_t;

uint8_t MPU6xx0_AutoDetect(MPU6xx0_t *dev);

MPU6xx0_Status MPU6xx0_Init(MPU6xx0_t *dev, uint8_t devadd);

MPU6xx0_Status MPU6xx0_Config(MPU6xx0_t *dev, MPU6xx0_Configuration_t *config);

void MPU6xx0_getValues(MPU6xx0_t *dev);

void MPU6xx0_getValuesIngIns(MPU6xx0_t *dev, MPU6xx0_Configuration_t *config);

bool mpu6xx0_ReadValues_Polling(MPU6xx0_t *dev);

bool mpu6xx0_ReadValues_DMA(MPU6xx0_t *dev);

bool mpu6xx0_ReadInterruptBits(MPU6xx0_t *dev, MPU6xx0_Configuration_t *config);

bool mpu6xx0_ReadFIFO(MPU6xx0_t *dev, uint8_t data[]);

#endif /* INC_MPU6500_MPU6050_H_ */
