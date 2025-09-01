/*
 * mpu6050.c
 *
 *  Created on: Sep 1, 2025
 *      Author: Enes
 */


#include "mpu6050.h"
#include <stdlib.h>

typedef enum{
	i2c_addr_0=0xD0,
	i2c_addr_1=0xD2
}mpu6050_i2c_addr;

typedef enum{
	who_am_i_0=0x68,
	who_am_i_1=0x70
}mpu6050_who_am_i;

typedef enum {
	MPU6050_WHO_AM_I     = 0x75,

	MPU6050_XA_OFFSET_H  = 0x06,
	MPU6050_XA_OFFSET_L  = 0x07,
	MPU6050_YA_OFFSET_H  = 0x08,
	MPU6050_YA_OFFSET_L  = 0x09,
	MPU6050_ZA_OFFSET_H  = 0x0A,
	MPU6050_ZA_OFFSET_L  = 0x0B,
	MPU6050_XG_OFFSET_H  = 0x13,
	MPU6050_XG_OFFSET_L  = 0x14,
	MPU6050_YG_OFFSET_H  = 0x15,
	MPU6050_YG_OFFSET_L  = 0x16,
	MPU6050_ZG_OFFSET_H  = 0x17,
	MPU6050_ZG_OFFSET_L  = 0x18,
    MPU6050_SAMPLRT      = 0x19, // Sample Rate Divider
    MPU6050_CONFIG       = 0x1A, // General config (DLPF etc.)
    MPU6050_GYRO_CONFIG  = 0x1B, // Gyro config
    MPU6050_ACCEL_CONFIG = 0x1C, // Accelerometer config
	MPU6050_ACCEL_CONFIG2 = 0x1D, // Accelerometer config

    MPU6050_PWR_MGMT_1   = 0x6B, // Power management 1
    MPU6050_PWR_MGMT_2   = 0x6C, // Power management 2

    MPU6050_TEMP_OUT_H   = 0x41, // Temperature high byte
    MPU6050_GYRO_XOUT_H  = 0x43, // Gyro X high byte
    MPU6050_ACCEL_XOUT_H = 0x3B, // Accel X high byte

    MPU6050_INT_PIN      = 0x37, // INT pin config
    MPU6050_INT_ENABLE   = 0x38, // Interrupt enable
    MPU6050_INT_STATUS   = 0x3A, // Interrupt status

    MPU6050_WAKE_ON_MOTION = 0x1F, // Wake-on-motion
    MPU6050_FIFO_ENABLE    = 0x23, // FIFO enable
	MPU6050_FIFO_COUNT      = 0x72, // FIFO read
    MPU6050_FIFO_READ      = 0x74, // FIFO read
    MPU6050_USER_CONTROL   = 0x6A  // User control (FIFO, I2C master, reset)
} MPU6050_RegisterMap;


struct mpu6050_t{
	I2C_HandleTypeDef* i2c_handle;
	uint8_t i2c_add;
	uint8_t who_am_i;
	uint32_t max_delay;

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
};

MPU6050_Status static write_register(mpu6050_handle_t dev, uint16_t MemAddress, uint8_t *pData, uint16_t Size){
	if(HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_add, MemAddress, 1, pData, Size, dev->max_delay)==HAL_OK){
		return MPU6050_OK;
	}
	return MPU6050_FAIL;
}

MPU6050_Status static read_register(mpu6050_handle_t dev, uint16_t MemAddress, uint8_t *pData, uint16_t Size){
	if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_add, MemAddress, 1, pData, Size, dev->max_delay)==HAL_OK){
		return MPU6050_OK;
	}
	return MPU6050_FAIL;
}

MPU6050_Status static read_register_dma(mpu6050_handle_t dev, uint16_t MemAddress, uint8_t *pData, uint16_t Size){
	if(HAL_I2C_Mem_Read_DMA(dev->i2c_handle, dev->i2c_add, MemAddress, 1, pData, Size)==HAL_OK){
		return MPU6050_OK;
	}
	return MPU6050_FAIL;
}

uint8_t mpu6050_autodetect(I2C_HandleTypeDef* hi2c){
	if(HAL_I2C_IsDeviceReady(hi2c, i2c_addr_0, 1, 1000)==HAL_OK){
		return i2c_addr_0;
	}else if(HAL_I2C_IsDeviceReady(hi2c, i2c_addr_1, 1, 1000)==HAL_OK){
		return i2c_addr_1;
	}else{
		return 0;
	}
}

mpu6050_handle_t mpu6050_init(mpu6050_config_t* config, uint8_t i2c_address){
	if(i2c_address!=i2c_addr_0 && i2c_address!=i2c_addr_1) return NULL;
	mpu6050_handle_t dev = (mpu6050_handle_t)malloc(sizeof(struct mpu6050_t));
	if(dev==NULL) return NULL;
	dev->i2c_handle=config->i2c_handle;
	dev->i2c_add=i2c_address;
	dev->max_delay=config->max_delay;

	if(read_register(dev, MPU6050_WHO_AM_I, &dev->who_am_i, 1)!=MPU6050_OK){
		return NULL;
	}
	if(dev->who_am_i!=who_am_i_0 && dev->who_am_i!=who_am_i_1){
		return NULL;
	}
	uint8_t wake_up_cmd=0;
	if(write_register(dev, MPU6050_PWR_MGMT_1, &wake_up_cmd, 1)!=MPU6050_OK){
		return NULL;
	}
	HAL_Delay(100); // reset sonrası 100 ms bekleme
	return dev;
}


MPU6050_Status mpu6050_configurate(mpu6050_handle_t dev, mpu6050_config_t* config){
	if(dev->i2c_add!=i2c_addr_0 && dev->i2c_add!=i2c_addr_1) return MPU6050_FAIL;
	if(write_register(dev, MPU6050_PWR_MGMT_1, &config->pwr_mgmt1.raw, 1)!=MPU6050_OK){
		return REGISTER_WRITE_UNAVAILABLE;
	}
	if(write_register(dev, MPU6050_CONFIG, &config->config.raw, 1)!=MPU6050_OK){
		return REGISTER_WRITE_UNAVAILABLE;
	}
	if(write_register(dev, MPU6050_GYRO_CONFIG, &config->gyro_config.raw, 1)!=MPU6050_OK){
		return REGISTER_WRITE_UNAVAILABLE;
	}
	if(write_register(dev, MPU6050_ACCEL_CONFIG, &config->accel_config.raw, 1)!=MPU6050_OK){
		return REGISTER_WRITE_UNAVAILABLE;
	}
	if(write_register(dev, MPU6050_ACCEL_CONFIG2, &config->accel_config2.raw, 1)!=MPU6050_OK){
		return REGISTER_WRITE_UNAVAILABLE;
	}
	if(write_register(dev, MPU6050_SAMPLRT, &config->sample_rate, 1)!=MPU6050_OK){
		return REGISTER_WRITE_UNAVAILABLE;
	}
	if(write_register(dev, MPU6050_INT_PIN, &config->int_pin_config.raw, 1)!=MPU6050_OK){
		return REGISTER_WRITE_UNAVAILABLE;
	}
	if(write_register(dev, MPU6050_INT_ENABLE, &config->int_en.raw, 1)!=MPU6050_OK){
		return REGISTER_WRITE_UNAVAILABLE;
	}
	if(write_register(dev, MPU6050_INT_STATUS, &config->int_stat.raw, 1)!=MPU6050_OK){
		return REGISTER_WRITE_UNAVAILABLE;
	}
	if(write_register(dev, MPU6050_USER_CONTROL, &config->user_ctrl.raw, 1)!=MPU6050_OK){
		return REGISTER_WRITE_UNAVAILABLE;
	}
	if(write_register(dev, MPU6050_FIFO_ENABLE, &config->fifo_en.raw, 1)!=MPU6050_OK){
		return REGISTER_WRITE_UNAVAILABLE;
	}
	return MPU6050_OK;
}

void inline mpu6050_get_values(mpu6050_handle_t dev){
	int16_t tempData;
	dev->accel_G[0] = (int16_t) ((dev->raw_data[0] << 8) | dev->raw_data[1]);
	dev->accel_G[1] = (int16_t) ((dev->raw_data[2] << 8) | dev->raw_data[3]);
	dev->accel_G[2] = (int16_t) ((dev->raw_data[4] << 8) | dev->raw_data[5]);
	dev->gyro_dps[0] = (int16_t) ((dev->raw_data[6] << 8) | dev->raw_data[7]);
	dev->gyro_dps[1] = (int16_t) ((dev->raw_data[8] << 8) | dev->raw_data[9]);
	dev->gyro_dps[2] = (int16_t) ((dev->raw_data[10] << 8) | dev->raw_data[11]);
	tempData = (int16_t) ((dev->raw_data[12] << 8) | dev->raw_data[13]);
	if (dev->who_am_i == who_am_i_1) {
		dev->temperature_C = (float) tempData / 333.87 + 21.00;
	} else if (dev->who_am_i == who_am_i_0) {
		dev->temperature_C = (float) tempData / 340.0 + 36.53;
	}
}

void inline mpu6050_get_values_ing_ins(mpu6050_handle_t dev,
		mpu6050_config_t *config) {

	dev->accelIng[0] = (float) ((dev->accel_G[0] / 16384.0)
			/ (config->accel_config.bits.accel_fs_sel + 1));
	dev->accelIng[1] = (float) ((dev->accel_G[1] / 16384.0)
			/ (config->accel_config.bits.accel_fs_sel + 1));
	dev->accelIng[2] = (float) ((dev->accel_G[2] / 16384.0)
			/ (config->accel_config.bits.accel_fs_sel + 1));

	dev->gyroIns[0] = (float) ((dev->gyro_dps[0] / 131.0)
			/ (config->gyro_config.bits.gyro_fs_sel + 1));
	dev->gyroIns[1] = (float) ((dev->gyro_dps[1] / 131.0)
			/ (config->gyro_config.bits.gyro_fs_sel + 1));
	dev->gyroIns[2] = (float) ((dev->gyro_dps[2] / 131.0)
			/ (config->gyro_config.bits.gyro_fs_sel + 1));

}

MPU6050_Status mpu6050_read_values_poll(mpu6050_handle_t dev){
	if(read_register(dev, MPU6050_ACCEL_XOUT_H, dev->raw_data, 14)==MPU6050_OK){
		return MPU6050_OK;
	}
	return REGISTER_READ_UNAVAILABLE;
}

MPU6050_Status mpu6050_read_values_dma(mpu6050_handle_t dev){
	if(read_register_dma(dev, MPU6050_ACCEL_XOUT_H, dev->raw_data, 14)==MPU6050_OK){
		return MPU6050_OK;
	}
	return REGISTER_READ_UNAVAILABLE;
}

MPU6050_Status mpu6050_read_fifo_dma(mpu6050_handle_t dev, uint8_t* data_buffer, uint16_t buffer_size){
	uint8_t fifo_size[2];
	if(read_register(dev, MPU6050_FIFO_COUNT, fifo_size, 2)!=MPU6050_OK){
		return REGISTER_READ_UNAVAILABLE;
	}
	uint16_t data_size = (fifo_size[0]<<8) | fifo_size[1];
	if(data_size == 0) return FIFO_EMPTY;
	uint16_t read_size = (buffer_size > data_size) ? data_size : buffer_size;
	if(read_register_dma(dev, MPU6050_FIFO_READ, data_buffer, read_size)!=MPU6050_OK){
		return REGISTER_READ_UNAVAILABLE;
	}
	return MPU6050_OK;
}
