#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#define DRIVER_NAME "mpu6050"

/* MPU6050 Registers */
#define MPU6050_PWR_MGMT1   0x6B
#define MPU6050_WHO_AM_I    0x75
#define MPU6050_ACCEL_XOUT  0x3B

/* ioctl commands */
#define MPU6050_SET_ACCEL_RANGE _IOW('m', 1, int)
#define MPU6050_SET_GYRO_RANGE  _IOW('m', 2, int)

/* Sensor data */
struct mpu6050_data {
    s16 ax, ay, az;
    s16 gx, gy, gz;
};

struct mpu6050_dev {
    struct i2c_client *client;
    struct mutex lock;

    dev_t dev_num;
    struct cdev cdev;
    struct class *class;
};

/* ---------------- I2C Helpers ---------------- */

static int mpu6050_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
    return i2c_smbus_write_byte_data(client, reg, val);
}

static int mpu6050_read_burst(struct i2c_client *client,
                              u8 reg, u8 *buf, int len)
{
    struct i2c_msg msgs[2] = {
        {
            .addr  = client->addr,
            .flags = 0,
            .len   = 1,
            .buf   = &reg,
        },
        {
            .addr  = client->addr,
            .flags = I2C_M_RD,
            .len   = len,
            .buf   = buf,
        }
    };

    return i2c_transfer(client->adapter, msgs, 2);
}

/* ---------------- File Operations ---------------- */

static int mpu6050_open(struct inode *inode, struct file *file)
{
    struct mpu6050_dev *mpu =
        container_of(inode->i_cdev, struct mpu6050_dev, cdev);
    file->private_data = mpu;
    return 0;
}

static ssize_t mpu6050_read(struct file *file,
                            char __user *buf,
                            size_t count, loff_t *ppos)
{
    struct mpu6050_dev *mpu = file->private_data;
    struct mpu6050_data data;
    u8 raw[14];

    if (count < sizeof(data))
        return -EINVAL;

    mutex_lock(&mpu->lock);

    if (mpu6050_read_burst(mpu->client, MPU6050_ACCEL_XOUT, raw, 14) != 2) {
        mutex_unlock(&mpu->lock);
        return -EIO;
    }

    data.ax = (raw[0] << 8) | raw[1];
    data.ay = (raw[2] << 8) | raw[3];
    data.az = (raw[4] << 8) | raw[5];

    data.gx = (raw[8]  << 8) | raw[9];
    data.gy = (raw[10] << 8) | raw[11];
    data.gz = (raw[12] << 8) | raw[13];

    mutex_unlock(&mpu->lock);

    if (copy_to_user(buf, &data, sizeof(data)))
        return -EFAULT;

    return sizeof(data);
}

static long mpu6050_ioctl(struct file *file,
                          unsigned int cmd,
                          unsigned long arg)
{
    struct mpu6050_dev *mpu = file->private_data;
    int val;

    if (copy_from_user(&val, (int __user *)arg, sizeof(int)))
        return -EFAULT;

    mutex_lock(&mpu->lock);

    switch (cmd) {
    case MPU6050_SET_ACCEL_RANGE:
        mpu6050_write_reg(mpu->client, 0x1C, val);
        break;
    case MPU6050_SET_GYRO_RANGE:
        mpu6050_write_reg(mpu->client, 0x1B, val);
        break;
    default:
        mutex_unlock(&mpu->lock);
        return -EINVAL;
    }

    mutex_unlock(&mpu->lock);
    return 0;
}

static const struct file_operations mpu6050_fops = {
    .owner          = THIS_MODULE,
    .open           = mpu6050_open,
    .read           = mpu6050_read,
    .unlocked_ioctl = mpu6050_ioctl,
};

/* ---------------- Probe & Remove ---------------- */

static int mpu6050_probe(struct i2c_client *client)
{
    struct mpu6050_dev *mpu;
    u8 whoami;
    int ret;

    dev_info(&client->dev, "mpu6050 probe entered (DT based)\n");

    mpu = devm_kzalloc(&client->dev, sizeof(*mpu), GFP_KERNEL);
    if (!mpu)
        return -ENOMEM;

    mpu->client = client;
    mutex_init(&mpu->lock);
    i2c_set_clientdata(client, mpu);

    /* Wake up sensor */
    mpu6050_write_reg(client, MPU6050_PWR_MGMT1, 0x00);
    msleep(10);

    whoami = i2c_smbus_read_byte_data(client, MPU6050_WHO_AM_I);
    if (whoami != 0x68) {
        dev_err(&client->dev, "WHO_AM_I mismatch: 0x%x\n", whoami);
        return -ENODEV;
    }

    /* Character device creation */
    ret = alloc_chrdev_region(&mpu->dev_num, 0, 1, "mpu6050");
    if (ret)
        return ret;

    cdev_init(&mpu->cdev, &mpu6050_fops);
    ret = cdev_add(&mpu->cdev, mpu->dev_num, 1);
    if (ret)
        goto err_chrdev;

    mpu->class = class_create(THIS_MODULE, "mpu6050");
    if (IS_ERR(mpu->class)) {
        ret = PTR_ERR(mpu->class);
        goto err_cdev;
    }

    if (!device_create(mpu->class, NULL, mpu->dev_num, NULL, "mpu6050")) {
        ret = -ENODEV;
        goto err_class;
    }

    dev_info(&client->dev, "/dev/mpu6050 created\n");
    return 0;

err_class:
    class_destroy(mpu->class);
err_cdev:
    cdev_del(&mpu->cdev);
err_chrdev:
    unregister_chrdev_region(mpu->dev_num, 1);
    return ret;
}

static int mpu6050_remove(struct i2c_client *client)
{
    struct mpu6050_dev *mpu = i2c_get_clientdata(client);

    device_destroy(mpu->class, mpu->dev_num);
    class_destroy(mpu->class);
    cdev_del(&mpu->cdev);
    unregister_chrdev_region(mpu->dev_num, 1);

    dev_info(&client->dev, "mpu6050 removed\n");
    return 0;
}

/* ---------------- DT Match Table ---------------- */

static const struct of_device_id mpu6050_of_match[] = {
    { .compatible = "SenseEdge,mpu6050" },
    { }
};
MODULE_DEVICE_TABLE(of, mpu6050_of_match);

/* ---------------- I2C Driver ---------------- */

static struct i2c_driver mpu6050_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = mpu6050_of_match,
    },
    .probe_new  = mpu6050_probe,
    .remove = mpu6050_remove,
};

module_i2c_driver(mpu6050_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Riddhi Sawarkar");
MODULE_DESCRIPTION("MPU6050 DT-based Embedded Linux Driver");
