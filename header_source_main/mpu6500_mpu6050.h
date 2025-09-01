/*
 * mpu6050.h
 *
 *  Created on: Sep 1, 2025
 *      Author: Enes
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"

struct mpu6050_t;
typedef struct mpu6050_t* mpu6050_handle_t;

typedef enum {
	MPU6050_OK = 0,
	MPU6050_FAIL,
	DEVICE_NOT_FOUND,
	WHO_IS_THIS_DEVICE,
	REGISTER_READ_UNAVAILABLE,
	REGISTER_WRITE_UNAVAILABLE,
	UNINITED_DEVICE,
	FIFO_EMPTY
} MPU6050_Status;

typedef enum {
	GYRO_250_DPS = 0, GYRO_500_DPS = 1, GYRO_1000_DPS = 2, GYRO_2000_DPS = 3,
} MPU6050_gyro_config;

typedef enum {
	ACCEL_2G = 0, ACCEL_4G = 1, ACCEL_8G = 2, ACCEL_16G = 3,
} MPU6050_accel_config;


typedef union{
	uint8_t raw;
	struct{
		uint8_t dlpf_cfg: 3;
		uint8_t ext_sync_set: 3;
		uint8_t fifo_mode: 1;
		uint8_t reserved: 1;
	}bits;
}mpu6050_reg_config;

typedef union{
	uint8_t raw;
	struct{
		uint8_t f_choice_b: 2;
		uint8_t reserved: 1;
		uint8_t gyro_fs_sel: 2;
		uint8_t self_test_z: 1;
		uint8_t self_test_y: 1;
		uint8_t self_test_x: 1;
	}bits;
}mpu6050_reg_gyro_config;

typedef union{
	uint8_t raw;
	struct{
		uint8_t reserved: 3;
		uint8_t accel_fs_sel: 2;
		uint8_t self_test_z: 1;
		uint8_t self_test_y: 1;
		uint8_t self_test_x: 1;
	}bits;
}mpu6050_reg_accel_config;

typedef union{
	uint8_t raw;
	struct{
		uint8_t a_dlpf_cfg: 3;
		uint8_t accel_fchoice_b: 1;
		uint8_t reserved: 4;
	}bits;
}mpu6050_reg_accel_config2;

typedef union{
	uint8_t raw;
	struct{
		uint8_t clk_sel: 3;
		uint8_t temp_dis: 1;
		uint8_t gyro_standby: 1;
		uint8_t cycle: 1;
		uint8_t sleep: 1;
		uint8_t device_reset: 1;
	}bits;
}mpu6050_reg_pwr_mgmt1;

typedef union{
	uint8_t raw;
	struct{
		uint8_t reserved: 1;
		uint8_t bypass_en: 1;
		uint8_t fsync_int_mode_en: 1;
		uint8_t actl_fsync: 1;
		uint8_t int_anyrd_2clear: 1;
		uint8_t latch_int_en: 1;
		uint8_t open: 1;
		uint8_t actl: 1;
	}bits;
}mpu6050_reg_int_pin_config;

typedef union{
	uint8_t raw;
	struct{
		uint8_t raw_rdy_en: 1;
		uint8_t reserved: 2;
		uint8_t fsync_int_en: 1;
		uint8_t fifo_overflow_en: 1;
		uint8_t reserved2: 1;
		uint8_t wom_en: 1;
		uint8_t reserved3: 1;
	}bits;
}mpu6050_reg_int_en;

typedef union{
	uint8_t raw;
	struct{
		uint8_t raw_data_rdy_int: 1;
		uint8_t dmp_int: 1;
		uint8_t reserved: 1;
		uint8_t fsync_int: 1;
		uint8_t fifo_overflow_int: 1;
		uint8_t reserved2: 1;
		uint8_t wom_int: 1;
		uint8_t reserved3: 1;
	}bits;
}mpu6050_reg_int_stat;

typedef union{
	uint8_t raw;
	struct{
		uint8_t sig_cond_rst: 1;
		uint8_t i2c_mst_rst: 1;
		uint8_t fifo_rst: 1;
		uint8_t dmp_rst: 1;
		uint8_t i2c_if_dis: 1;
		uint8_t i2c_mst_en: 1;
		uint8_t fifo_en: 1;
		uint8_t dmp_en: 1;
	}bits;
}mpu6050_reg_user_ctrl;

typedef union{
	uint8_t raw;
	struct{
		uint8_t slv_0: 1;
		uint8_t slv_1: 1;
		uint8_t slv_2: 1;
		uint8_t accel: 1;
		uint8_t gyro_zout: 1;
		uint8_t gyro_yout: 1;
		uint8_t gyro_xout: 1;
		uint8_t temp_out: 1;
	}bits;
}mpu6050_reg_fifo_en;

typedef struct{
	I2C_HandleTypeDef* i2c_handle;
	uint8_t i2c_add;
	uint32_t max_delay;
	uint8_t sample_rate;
	mpu6050_reg_config config;
	mpu6050_reg_gyro_config gyro_config;
	mpu6050_reg_accel_config accel_config;
	mpu6050_reg_accel_config2 accel_config2;
	mpu6050_reg_pwr_mgmt1 pwr_mgmt1;
	mpu6050_reg_int_pin_config int_pin_config;
	mpu6050_reg_int_en int_en;
	mpu6050_reg_int_stat int_stat;
	mpu6050_reg_user_ctrl user_ctrl;
	mpu6050_reg_fifo_en fifo_en;
}mpu6050_config_t;

uint8_t mpu6050_autodetect(I2C_HandleTypeDef* hi2c);
mpu6050_handle_t mpu6050_init(mpu6050_config_t* config, uint8_t i2c_address);
MPU6050_Status mpu6050_configurate(mpu6050_handle_t dev, mpu6050_config_t* config);
void mpu6050_get_values(mpu6050_handle_t dev);
void mpu6050_get_values_ing_ins(mpu6050_handle_t dev, mpu6050_config_t *config);
MPU6050_Status mpu6050_read_values_poll(mpu6050_handle_t dev);
MPU6050_Status mpu6050_read_values_dma(mpu6050_handle_t dev);


#endif /* INC_MPU6050_H_ */
