/* ================= MPU6050 Register Map ================= */

#define MPU6050_I2C_ADDR 0x68

#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT1 0x6B
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CFG 0x1B
#define MPU6050_ACCEL_CFG 0x1C

#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_STATUS 0x3A

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_ZOUT_H 0x3F

#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_ZOUT_H 0x47

static int mpu6050_write_reg(struct i2c_client *client,
                             u8 reg, u8 val)
{
    int ret;

    ret = i2c_smbus_write_byte_data(client, reg, val);
    if (ret < 0)
        dev_err(&client->dev,
                "MPU6050: write failed reg=0x%x\n", reg);

    return ret;
}

static int mpu6050_read_reg(struct i2c_client *client,
                            u8 reg)
{
    int ret;

    ret = i2c_smbus_read_byte_data(client, reg);
    if (ret < 0)
        dev_err(&client->dev,
                "MPU6050: read failed reg=0x%x\n", reg);

    return ret;
}

static int mpu6050_read_burst(struct i2c_client *client,
                              u8 start_reg,
                              u8 *buf,
                              int len)
{
    struct i2c_msg msgs[2] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = &start_reg,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = len,
            .buf = buf,
        }};

    int ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret != 2)
    {
        dev_err(&client->dev,
                "MPU6050: burst read failed\n");
        return -EIO;
    }

    return 0;
}

static int mpu6050_wakeup(struct i2c_client *client)
{
    /* Clear sleep bit */
    return mpu6050_write_reg(client,
                             MPU6050_PWR_MGMT1,
                             0x00);
}

static int mpu6050_check_whoami(struct i2c_client *client)
{
    int id = mpu6050_read_reg(client, MPU6050_WHO_AM_I);

    if (id != 0x68)
    {
        dev_err(&client->dev,
                "MPU6050: WHO_AM_I mismatch (0x%x)\n", id);
        return -ENODEV;
    }

    return 0;
}
