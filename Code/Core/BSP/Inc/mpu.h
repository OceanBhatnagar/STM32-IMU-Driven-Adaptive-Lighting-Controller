/*
 * mpu.h
 *
 *  Created on: 10-Feb-2026
 *      Author: Ocean
 */

#ifndef BSP_INC_MPU_H_
#define BSP_INC_MPU_H_

#include<stm32f4xx_hal.h>
#include<math.h>

extern I2C_HandleTypeDef hi2c2;


//ACCELEROMETER DATA STRUCTURE
typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}MPU_ACCEL_t;

//GYROSCOPE DATA STRUCTURE
typedef struct{
	int16_t x;
	int y;
	int z;
}MPU_GYRO_t;

void MPU_6050_INIT(void);
MPU_ACCEL_t* MPU_READ_ACCELEROMETER(MPU_ACCEL_t *acel_data);
MPU_GYRO_t* MPU_READ_GYROMETER(MPU_GYRO_t *gyro_data);

MPU_ACCEL_t ACCELEROMETER_CALIBRATION(MPU_ACCEL_t *acel_data,const MPU_ACCEL_t *error_offset);
MPU_GYRO_t GYROSCOPE_CALIBRATION(MPU_GYRO_t *gyro_data,const MPU_GYRO_t *error_offset);

float roll_angle(MPU_ACCEL_t *acel_data);
void MPU_read_burst(uint8_t start_reg, uint8_t *buffer, uint8_t len);

typedef enum{
	INT_LEVEL_ACTIVE_HIGH=0x00,
	INT_LEVEL_ACTIVE_LOW,
}MPU_Interrupt_config_t;

typedef enum{
	RAW_RDY_INT=0x01,
	I2C_MST_INT=0x08,
	FIFO_OFLOW_INT=0x10,
	MOT_INT=0x40,
	ALL_INT=0xFF
}mpu_interrupt_t;

void MPU_Interrupt_config(MPU_Interrupt_config_t level);
void MPU_Interrupt_Enable(mpu_interrupt_t);
void MPU_Interrupt_Disable(mpu_interrupt_t);
void mpu_interrupt_handle(void);
#define MPU_ADDR          (0x68<<1)

#define REG_WHO_AM_I     0x75
#define REG_POWER_MGMT1   0x6B
#define MPU_REG_ACCEL     0x3B
#define MPU_REG_GYRO      0X43

#define MPU6050_REG_INT_STATUS     		 0x3A
#define MPU6050_REG_INT_EN         		 0x38
#define MPU6050_REG_MOT_THR 			 0x1F
#define MPU6050_REG_MOT_DUR 			 0x20
#define MPU6050_REG_INT_PIN_CFG      	 0x37

#endif /* BSP_INC_MPU_H_ */
