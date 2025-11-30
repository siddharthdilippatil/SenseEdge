/*
 * mpu6050.c
 *
 *  Created on: Nov 27, 2025
 *      Author: siddharthpatil
 */

#include "mpu6050.h"

void mpu6050_init(void)
{
    uint8_t check;
    i2c_start();
    i2c_send_slave_addr(MPU_SLAVE_ADDR_W);
    i2c_send_data(WHO_AM_I);
    i2c_rep_start();
    i2c_send_slave_addr(MPU_SLAVE_ADDR_R);
    check = i2c_rec_nack();

    if (check == 0x68)
    {
        i2c_start();
        i2c_send_slave_addr(MPU_SLAVE_ADDR_W);
        i2c_send_data(PWR_MGMT_1);
        i2c_send_data(0x00); // power on mpu from sleep mode
        i2c_stop();

        i2c_start();
        i2c_send_slave_addr(MPU_SLAVE_ADDR_W);
        i2c_send_data(SMPLRT_DIV);
        i2c_send_data(0x07); // write 7 to smplrt_div to get 1khz sample rate
        i2c_stop();

        i2c_start();
        i2c_send_slave_addr(MPU_SLAVE_ADDR_W);
        i2c_send_data(GYRO_CONFIG);
        i2c_send_data(0x00); // Wite 00 to set to full scale range of +-250 degree in gyro
        i2c_stop();

        i2c_start();
        i2c_send_slave_addr(MPU_SLAVE_ADDR_W);
        i2c_send_data(ACCEL_CONFIG);
        i2c_send_data(0x00); // Write 00 to set to full scale range of +-2g in accel
        i2c_stop();
    }
}

void mpu6050_burst_read(mpu_raw_t *raw_data)
{
    uint8_t high, low;
    i2c_start();
    i2c_send_slave_addr(MPU_SLAVE_ADDR_W);
    i2c_send_data(ACCEL_XOUT_H);
    i2c_rep_start();
    i2c_send_slave_addr(MPU_SLAVE_ADDR_R);

    high = i2c_rec_ack();
    low = i2c_rec_ack();
    raw_data->accel_x = (int16_t)((high << 8) | (low));

    high = i2c_rec_ack();
    low = i2c_rec_ack();
    raw_data->accel_y = (int16_t)((high << 8) | (low));

    high = i2c_rec_ack();
    low = i2c_rec_ack();
    raw_data->accel_z = (int16_t)((high << 8) | (low));

    high = i2c_rec_ack();
    low = i2c_rec_ack();
    raw_data->accel_temp = (int16_t)((high << 8) | (low));

    high = i2c_rec_ack();
    low = i2c_rec_ack();
    raw_data->gyro_x = (int16_t)((high << 8) | (low));

    high = i2c_rec_ack();
    low = i2c_rec_ack();
    raw_data->gyro_y = (int16_t)((high << 8) | (low));

    high = i2c_rec_ack();
    low = i2c_rec_nack(); // includes stop condition
    raw_data->gyro_z = (int16_t)((high << 8) | (low));
}
