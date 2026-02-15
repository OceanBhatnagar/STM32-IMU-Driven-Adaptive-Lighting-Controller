/*
 * mpu.c
 *
 *  Created on: 10-Feb-2026
 *      Author: Ocean
 */


#include"mpu.h"
#include"main.h"
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include<task.h>
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;
extern osThreadId_t SensorReadTaskHandle;
#define I2C_TIMEOUT 500UL

void MPU_6050_INIT(void){

	//READ 1 BYTE FROM SENSOR
	uint8_t response=0;
	HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, REG_WHO_AM_I,1, &response, 1, 500);

	//WRITE BYTE TO WAKE SENSOR FROM SLEEP MODE
	uint8_t data=0;
	if(HAL_I2C_Mem_Write(&hi2c2,MPU_ADDR,REG_POWER_MGMT1, I2C_MEMADD_SIZE_8BIT,&data,1,500)  != HAL_OK){
			Error_Handler();
		}


    // 3. Configure accelerometer to ±2g (most sensitive)
    data = 0x00;  // ±2g range
    HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x1C,  // ACCEL_CONFIG register
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 500);

    // 4. Configure gyroscope to ±250°/s (most sensitive)
    data = 0x00;  // ±250°/s range
    HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x1B,  // GYRO_CONFIG register
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 500);

    // 5. Set sample rate to 100Hz
    data = 0x09;  // Sample Rate = 1kHz / (1 + 9) = 100Hz
    HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x19,  // SMPLRT_DIV register
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 500);

    // 6. Configure Digital Low Pass Filter (DLPF)
    data = 0x03;  // DLPF ~44Hz bandwidth (reduces noise)
    HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x1A,  // CONFIG register
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 500);

    HAL_Delay(50);  // Final stabilization delay
}


MPU_ACCEL_t* MPU_READ_ACCELEROMETER(MPU_ACCEL_t *acel_data){
	uint8_t raw_data[6];
	MPU_read_burst(MPU_REG_ACCEL,(uint8_t*)&raw_data,sizeof(raw_data));
	acel_data->x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
	acel_data->y=(int16_t)raw_data[2]<<8 | raw_data[3];
	acel_data->z=(int16_t)raw_data[4]<<8 | raw_data[5];

	return acel_data;
}

MPU_GYRO_t* MPU_READ_GYROMETER(MPU_GYRO_t *gyro_data){
	uint8_t raw_data[6];
	MPU_read_burst(MPU_REG_GYRO,(uint8_t*)&raw_data,sizeof(raw_data));

	gyro_data->x=(int16_t)raw_data[0]<<8 | raw_data[1];
	gyro_data->y=(int16_t)raw_data[2]<<8 | raw_data[3];
	gyro_data->z=(int16_t)raw_data[4]<<8 | raw_data[5];

	return gyro_data;
}

void MPU_read_burst(uint8_t start_reg, uint8_t *buffer, uint8_t len)
{
	HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, start_reg,
	                 I2C_MEMADD_SIZE_8BIT,
	                 buffer,len,500);
}

float roll_angle(MPU_ACCEL_t *acel_data){
	float angle=atan2(acel_data->y,acel_data->z)*(180/M_PI);
	return angle;
}

void mpu6050_read_byte(uint8_t reg_addr, uint8_t *data) {
	HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, reg_addr, 1 , data, 1, I2C_TIMEOUT);
}

void mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
 HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, reg_addr, 1 , &data, 1, I2C_TIMEOUT);
}

void MPU_Interrupt_config(MPU_Interrupt_config_t level){
	uint8_t int_cfg=0;

	MPU_read_burst(MPU6050_REG_INT_PIN_CFG ,&int_cfg,sizeof(int_cfg));


	int_cfg &=~0x80;
	int_cfg |= (uint8_t)level;

	mpu6050_write_byte(MPU6050_REG_INT_PIN_CFG,int_cfg);
}

void MPU_Interrupt_Enable(mpu_interrupt_t interrupt){
	uint8_t current_int_settings=0;
	mpu6050_read_byte(MPU6050_REG_INT_EN,&current_int_settings);
	current_int_settings |=(uint8_t)interrupt;
	mpu6050_write_byte(MPU6050_REG_INT_EN,current_int_settings);

}

void MPU_Interrupt_Disable(mpu_interrupt_t interrupt){
	uint8_t current_int_settings=0;
	if(interrupt != (uint8_t)ALL_INT){
		mpu6050_read_byte(MPU6050_REG_INT_EN,&current_int_settings);
	}

	current_int_settings &=~interrupt;
	mpu6050_write_byte(MPU6050_REG_INT_EN,current_int_settings);
}
void mpu_interrupt_handle()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Check if the task handle is valid
    if (SensorReadTaskHandle != NULL)
    {
        // 1. Wake up the sensor task immediately
        vTaskNotifyGiveFromISR(SensorReadTaskHandle, &xHigherPriorityTaskWoken);

        // 2. Force the CPU to switch to the sensor task now
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


