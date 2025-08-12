/*
 * mpu6500_mpu6050.c
 *
 *  Created on: Aug 11, 2025
 *      Author: Enes
 */

#include "mpu6500_mpu6050.h"

static MPU6xx0_Status _mpu6xx0readregister(MPU6xx0_t *dev, uint16_t MemAddress,
		uint8_t *pData, uint16_t Size) {

	if (HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_add, MemAddress,
	I2C_MEMADD_SIZE_8BIT, pData, Size, HAL_MAX_DELAY) == HAL_OK) {
		return MPU6xx0_OK;
	}

	return REGISTER_READ_UNAVAILABLE;
}

static MPU6xx0_Status _mpu6xx0readregister_DMA(MPU6xx0_t *dev,
		uint16_t MemAddress, uint8_t *pData, uint16_t Size) {

	if (HAL_I2C_Mem_Read_DMA(dev->i2c_handle, dev->i2c_add, MemAddress,
	I2C_MEMADD_SIZE_8BIT, pData, Size) == HAL_OK) {
		return MPU6xx0_OK;
	}

	return REGISTER_READ_UNAVAILABLE;
}

static MPU6xx0_Status _mpu6xx0writeregister(MPU6xx0_t *dev, uint16_t MemAddress,
		uint8_t *pData, uint16_t Size) {

	if (HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_add, MemAddress,
	I2C_MEMADD_SIZE_8BIT, pData, Size, HAL_MAX_DELAY) == HAL_OK) {
		return MPU6xx0_OK;
	}

	return REGISTER_READ_UNAVAILABLE;
}

uint8_t MPU6xx0_AutoDetect(MPU6xx0_t *dev) {
	for (uint8_t add = 0; add < 256; add++) {
		if (HAL_I2C_IsDeviceReady(dev->i2c_handle, add, 1,
		HAL_MAX_DELAY) == HAL_OK) {
			return add;;
		}
	}
	if (HAL_I2C_IsDeviceReady(dev->i2c_handle, MPU6xx0_I2C_ADDR_0, 1,
	HAL_MAX_DELAY) == HAL_OK) {
		return MPU6xx0_I2C_ADDR_0;
	} else if (HAL_I2C_IsDeviceReady(dev->i2c_handle, MPU6xx0_I2C_ADDR_1, 1,
	HAL_MAX_DELAY) == HAL_OK) {
		return MPU6xx0_I2C_ADDR_1;
	}
	return 0;
}

MPU6xx0_Status MPU6xx0_Init(MPU6xx0_t *dev, uint8_t devadd) {

	uint8_t databuffer;
	dev->i2c_add = devadd;
	if (dev->i2c_add != MPU6xx0_I2C_ADDR_0 && dev->i2c_add != MPU6xx0_I2C_ADDR_1) {
		databuffer = MPU6xx0_AutoDetect(dev);
		if (databuffer != 0) {
			dev->i2c_add = databuffer;
		} else {
			return DEVICE_NOT_FOUND;
		}
	}

	if (_mpu6xx0readregister(dev, MPU6xx0_WHO_AM_I_ADDR, &(dev->who_am_i), 1)
			!= MPU6xx0_OK) {
		return REGISTER_READ_UNAVAILABLE;
	}

	if (dev->who_am_i != MPU6050_WHO_AM_I && dev->who_am_i != MPU6500_WHO_AM_I) {
		return WHO_IS_THIS_DEVICE;
	}

	uint8_t wakeup_cmd = 0x00;
	if (_mpu6xx0writeregister(dev, MPU6xx0_PWR_MGMT_1, &wakeup_cmd, 1)
			!= MPU6xx0_OK) {
		return REGISTER_WRITE_UNAVAILABLE;
	}

	return MPU6xx0_OK;
}

MPU6xx0_Status MPU6xx0_Config(MPU6xx0_t *dev, MPU6xx0_Configuration_t *config) {

	if (dev->i2c_handle == NULL) {
		return UNINITED_DEVICE;
	}

	uint8_t pwr_mngmnt_buffer[2];
	uint8_t config_buffer[4];
	uint8_t int_config_buffer[2];
	uint8_t fifo_config_buffer;

	pwr_mngmnt_buffer[0] = ((config->Power_Manage1.Device_Reset << 7)
			| (config->Power_Manage1.Sleep << 6)
			| (config->Power_Manage1.Cycle << 5)
			| (config->Power_Manage1.Gyro_Standby << 4)
			| (config->Power_Manage1.Temp_Dis << 3)
			| (config->Power_Manage1.ClkSel));

	pwr_mngmnt_buffer[1] = ((config->Power_Manage2.Lp_Wake_Ctrl << 6)
			| (config->Power_Manage2.Disable_xa << 5)
			| (config->Power_Manage2.Disable_ya << 4)
			| (config->Power_Manage2.Disable_za << 3)
			| (config->Power_Manage2.Disable_xg << 2)
			| (config->Power_Manage2.Disable_yg << 1)
			| (config->Power_Manage2.Disable_zg));

	if (_mpu6xx0writeregister(dev, MPU6xx0_PWR_MGMT_1, pwr_mngmnt_buffer, 1)
			!= MPU6xx0_OK) {
		return REGISTER_WRITE_UNAVAILABLE;
	}

	config_buffer[0] = config->Sample_Rate;
	config_buffer[1] = ((0 << 7) | (config->Config.Fifo_Mode << 6)
			| (config->Config.Ext_Sync_Set << 3) | (config->Config.Dlpf_Cfg));
	config_buffer[2] = 0x00
			| ((config->Gyro_FS_Sel << 3) | (config->Fchoice_B));
	config_buffer[3] = 0x00
			| ((config->Accel_Fchoice_B << 3) | (config->A_Dlpf_Cfg));

	if (_mpu6xx0writeregister(dev, MPU6xx0_SAMPLRT_ADDR, config_buffer, 4)
			!= MPU6xx0_OK) {
		return REGISTER_WRITE_UNAVAILABLE;
	}

	int_config_buffer[0] = (config->Int_pin.Actl << 7)
			| (config->Int_pin.Open << 6) | (config->Int_pin.Latch_Int_En << 5)
			| (config->Int_pin.Int_Anyrd_2clear << 4)
			| (config->Int_pin.Actl_Fsync << 3)
			| (config->Int_pin.Fsync_Int_Mode_En << 2)
			| (config->Int_pin.Bypass_En << 1);
	int_config_buffer[1] = (config->Int_Enable.Wom_En << 6)
			| (config->Int_Enable.Fifo_OverFlow_En << 4)
			| (config->Int_Enable.Fsync_Int_En << 3)
			| (config->Int_Enable.RAW_RDY_EN);

	if (_mpu6xx0writeregister(dev, MPU6050_INT_PIN_ADDR, int_config_buffer, 2)
			!= MPU6xx0_OK) {
		return REGISTER_WRITE_UNAVAILABLE;
	}

	fifo_config_buffer = (config->FIFO_Enable.TEMP_OUT << 7)
			| (config->FIFO_Enable.GYRO_XOUT << 6)
			| (config->FIFO_Enable.GYRO_YOUT << 5)
			| (config->FIFO_Enable.GYRO_ZOUT << 4)
			| (config->FIFO_Enable.ACCEL << 3)
			| (config->FIFO_Enable.SLV_2 << 2)
			| (config->FIFO_Enable.SLV_1 << 1) | (config->FIFO_Enable.SLV_0);

	if (_mpu6xx0writeregister(dev, MPU6050_FIFO_ENABLE_ADDR,
			&fifo_config_buffer, 1) != MPU6xx0_OK) {
		return REGISTER_WRITE_UNAVAILABLE;
	}

	fifo_config_buffer=0x00;

	_mpu6xx0writeregister(dev, MPU6050_USER_CONTROL_ADDR, &fifo_config_buffer, 1);

	fifo_config_buffer=0x04;

	_mpu6xx0writeregister(dev, MPU6050_USER_CONTROL_ADDR, &fifo_config_buffer, 1);

	fifo_config_buffer=0x40;

	_mpu6xx0writeregister(dev, MPU6050_USER_CONTROL_ADDR, &fifo_config_buffer, 1);

	return MPU6xx0_OK;
}

void inline MPU6xx0_getValues(MPU6xx0_t *dev) {
	int16_t tempData;
	dev->accel_G[0] = (int16_t) ((dev->raw_data[0] << 8) | dev->raw_data[1]);
	dev->accel_G[1] = (int16_t) ((dev->raw_data[2] << 8) | dev->raw_data[3]);
	dev->accel_G[2] = (int16_t) ((dev->raw_data[4] << 8) | dev->raw_data[5]);
	dev->gyro_dps[0] = (int16_t) ((dev->raw_data[6] << 8) | dev->raw_data[7]);
	dev->gyro_dps[1] = (int16_t) ((dev->raw_data[8] << 8) | dev->raw_data[9]);
	dev->gyro_dps[2] = (int16_t) ((dev->raw_data[10] << 8) | dev->raw_data[11]);
	tempData = (int16_t) ((dev->raw_data[12] << 8) | dev->raw_data[13]);
	if (dev->who_am_i == MPU6500_WHO_AM_I) {
		dev->temperature_C = (float) tempData / 333.87 + 21.00;
	} else if (dev->who_am_i == MPU6050_WHO_AM_I) {
		dev->temperature_C = (float) tempData / 340.0 + 36.53;
	}
}

void inline MPU6xx0_getValuesIngIns(MPU6xx0_t *dev,
		MPU6xx0_Configuration_t *config) {

	dev->accelIng[0] = (float) ((dev->accel_G[0] / 16384.0)
			/ (config->Accel_FS_Sel + 1));
	dev->accelIng[1] = (float) ((dev->accel_G[1] / 16384.0)
			/ (config->Accel_FS_Sel + 1));
	dev->accelIng[2] = (float) ((dev->accel_G[2] / 16384.0)
			/ (config->Accel_FS_Sel + 1));

	dev->gyroIns[0] = (float) ((dev->gyro_dps[0] / 131.0)
			/ (config->Gyro_FS_Sel + 1));
	dev->gyroIns[1] = (float) ((dev->gyro_dps[1] / 131.0)
			/ (config->Gyro_FS_Sel + 1));
	dev->gyroIns[2] = (float) ((dev->gyro_dps[2] / 131.0)
			/ (config->Gyro_FS_Sel + 1));

}

bool mpu6xx0_ReadInterruptBits(MPU6xx0_t *dev, MPU6xx0_Configuration_t *config) {
	uint8_t databuffer;
	if (_mpu6xx0readregister(dev, MPU6050_INT_STATUS_ADDR, &databuffer, 1)
			!= MPU6xx0_OK) {
		return false;
	}
	config->Int_Stats.Wom_INT = databuffer & 0x40;
	config->Int_Stats.Fifo_OverFlow_INT = databuffer & 0x10;
	config->Int_Stats.FSYNC_INT = databuffer & 0x08;
	config->Int_Stats.DMP_INT = databuffer & 0x02;
	config->Int_Stats.RAW_DATA_RDY_INT = databuffer & 0x01;

	return true;
}

bool mpu6xx0_ReadFIFO(MPU6xx0_t *dev, uint8_t data[]) {
	if (_mpu6xx0readregister_DMA(dev, MPU6050_FIFO_READ_ADDR, data, sizeof(data))
			== MPU6xx0_OK) {
		return true;
	}
	return false;
}

bool mpu6xx0_ReadValues_DMA(MPU6xx0_t *dev) {
	if (_mpu6xx0readregister_DMA(dev, MPU6050_REG_ACCEL_XOUT_H, dev->raw_data,
			14) == MPU6xx0_OK) {
		return true;
	}
	return false;
}

bool mpu6xx0_ReadValues_Polling(MPU6xx0_t *dev) {
	if (_mpu6xx0readregister(dev, MPU6050_REG_ACCEL_XOUT_H, dev->raw_data, 14)
			== MPU6xx0_OK) {
		return true;
	}
	return false;
}
