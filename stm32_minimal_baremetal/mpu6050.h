/*
 * mpu6050.h
 *
 *  Created on: Nov 27, 2025
 *      Author: siddharthpatil
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f4xx.h"
#include "i2c.h"

#define MPU_SLAVE_ADDR_W 0xD0
#define MPU_SLAVE_ADDR_R 0xD1

#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

typedef struct mpu_raw_data
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t accel_temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu_raw_t;

void mpu6050_init(void);
void mpu6050_burst_read(mpu_raw_t *raw_data);

#endif /* MPU6050_H_ */
