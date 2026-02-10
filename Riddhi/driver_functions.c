/* ---------- IOCTL Handling ---------- */
/*
 *      Author: Riddhi Sawarkar
 */

static long mpu6050_ioctl(struct file *file,
                          unsigned int cmd,
                          unsigned long arg)
{
    struct mpu6050_dev *mpu = file->private_data;
    int val;

    if (!mpu || !mpu->present)
        return -ENODEV;

    if (copy_from_user(&val, (int __user *)arg, sizeof(int)))
        return -EFAULT;

    mutex_lock(&mpu->lock);

    switch (cmd)
    {
    case MPU6050_SET_ACCEL_RANGE:
        mpu->accel_range_g = val;
        break;

    case MPU6050_SET_GYRO_RANGE:
        mpu->gyro_range_dps = val;
        break;

    default:
        mutex_unlock(&mpu->lock);
        return -EINVAL;
    }

    /* Apply new configuration to hardware */
    mpu6050_apply_config(mpu);

    mutex_unlock(&mpu->lock);
    return 0;
}

/*
 * NOTE:
 * - Enables runtime sensor configuration
 * - Demonstrates kernel ↔ userspace control path
 * - Used by userspace app via ioctl()
 */

/* ---------- IRQ Handler ---------- */

static irqreturn_t mpu6050_irq_handler(int irq, void *dev_id)
{
    struct mpu6050_dev *mpu = dev_id;

    mpu->data_ready = true;
    wake_up_interruptible(&mpu->wq);

    return IRQ_HANDLED;
}

/*
 * NOTE:
 * - Triggered by MPU6050 INT pin
 * - Wakes up blocked read() in userspace
 * - No polling → interrupt-driven design
 */

/* ---------- Probe ---------- */

static int mpu6050_probe(struct i2c_client *client)
{
    struct mpu6050_dev *mpu;
    int ret;
    u8 whoami;

    pr_info("mpu6050: probe called\n");

    mpu = kzalloc(sizeof(*mpu), GFP_KERNEL);
    if (!mpu)
        return -ENOMEM;

    mutex_init(&mpu->lock);
    init_waitqueue_head(&mpu->wq);

    mpu->client = client;
    mpu->present = true;
    mpu->data_ready = false;

    i2c_set_clientdata(client, mpu);

    /* Wake up sensor */
    i2c_smbus_write_byte_data(client, MPU6050_PWR_MGMT1, 0x00);
    msleep(10);

    whoami = i2c_smbus_read_byte_data(client, MPU6050_WHO_AM_I);
    if (whoami != 0x68)
    {
        pr_err("mpu6050: WHO_AM_I mismatch\n");
        ret = -ENODEV;
        goto err_free;
    }

    /* Parse Device Tree properties */
    mpu6050_parse_dt(&client->dev, mpu);

    /* Apply initial configuration */
    mpu6050_apply_config(mpu);

    /* IRQ setup */
    mpu->irq = client->irq;
    if (mpu->irq > 0)
    {
        ret = request_irq(mpu->irq,
                          mpu6050_irq_handler,
                          IRQF_TRIGGER_RISING,
                          DRIVER_NAME,
                          mpu);
        if (ret)
            goto err_free;

        /* Enable Data Ready interrupt */
        i2c_smbus_write_byte_data(client, MPU6050_INT_ENABLE, 0x01);
    }

    pr_info("mpu6050: probe successful\n");
    return 0;

err_free:
    kfree(mpu);
    return ret;
}

/*
 * NOTE:
 * - Hardware validation (WHO_AM_I)
 * - DT parsing
 * - IRQ registration
 * - Sensor wake-up
 */

/* ---------- Remove ---------- */

static int mpu6050_remove(struct i2c_client *client)
{
    struct mpu6050_dev *mpu = i2c_get_clientdata(client);

    pr_info("mpu6050: remove called\n");

    if (mpu->irq > 0)
        free_irq(mpu->irq, mpu);

    mpu->present = false;
    kfree(mpu);

    return 0;
}

/*
 * NOTE:
 * - Frees IRQ
 * - Cleans device state
 * - Supports hot-unbind
 */

/* ---------- Power Management ---------- */

static int mpu6050_suspend(struct device *dev)
{
    struct mpu6050_dev *mpu = dev_get_drvdata(dev);

    pr_info("mpu6050: suspend\n");

    /* Put sensor into sleep mode */
    i2c_smbus_write_byte_data(mpu->client,
                              MPU6050_PWR_MGMT1,
                              0x40);
    return 0;
}

static int mpu6050_resume(struct device *dev)
{
    struct mpu6050_dev *mpu = dev_get_drvdata(dev);

    pr_info("mpu6050: resume\n");

    /* Wake sensor */
    i2c_smbus_write_byte_data(mpu->client,
                              MPU6050_PWR_MGMT1,
                              0x00);

    /* Re-apply configuration */
    mpu6050_apply_config(mpu);
    return 0;
}

static const struct dev_pm_ops mpu6050_pm_ops = {
    .suspend = mpu6050_suspend,
    .resume = mpu6050_resume,
};

/*
 * NOTE:
 * - Enables system suspend/resume
 * - Resume restores sensor configuration
 * - Interview-grade PM support
 */

static struct i2c_driver mpu6050_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = mpu6050_of_match,
        .pm = &mpu6050_pm_ops,
    },
    .probe_new = mpu6050_probe,
    .remove = mpu6050_remove,
};
