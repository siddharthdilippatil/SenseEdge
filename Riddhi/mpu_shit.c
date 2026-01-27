#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#define DRIVER_NAME "mpu6050"

/* MPU6050 Registers */
#define MPU6050_PWR_MGMT1    0x6B
#define MPU6050_WHO_AM_I     0x75
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

    int irq;
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

/* ---------------- File Ops ---------------- */

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

static int mpu6050_open(struct inode *inode, struct file *file)
{
    struct mpu6050_dev *mpu =
        container_of(inode->i_cdev, struct mpu6050_dev, cdev);
    file->private_data = mpu;
    return 0;
}

static const struct file_operations mpu6050_fops = {
    .owner          = THIS_MODULE,
    .open           = mpu6050_open,
    .read           = mpu6050_read,
    .unlocked_ioctl = mpu6050_ioctl,
};

/* ---------------- IRQ ---------------- */

static irqreturn_t mpu6050_irq_handler(int irq, void *dev_id)
{
    return IRQ_HANDLED;
}

/* ---------------- Probe & Remove (OLD API) ---------------- */

static int mpu6050_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    struct mpu6050_dev *mpu;
    u8 whoami;
    int ret;

    mpu = devm_kzalloc(&client->dev, sizeof(*mpu), GFP_KERNEL);
    if (!mpu)
        return -ENOMEM;

    mpu->client = client;
    mutex_init(&mpu->lock);
    i2c_set_clientdata(client, mpu);

    mpu6050_write_reg(client, MPU6050_PWR_MGMT1, 0x00);
    msleep(10);

    whoami = i2c_smbus_read_byte_data(client, MPU6050_WHO_AM_I);
    if (whoami != 0x68)
        return -ENODEV;

    ret = alloc_chrdev_region(&mpu->dev_num, 0, 1, "mpu6050");
if (ret) {
    dev_err(&client->dev, "alloc_chrdev_region failed\n");
    return ret;
}

cdev_init(&mpu->cdev, &mpu6050_fops);
ret = cdev_add(&mpu->cdev, mpu->dev_num, 1);
if (ret) {
    dev_err(&client->dev, "cdev_add failed\n");
    unregister_chrdev_region(mpu->dev_num, 1);
    return ret;
}

mpu->class = class_create(THIS_MODULE, "mpu6050");
if (IS_ERR(mpu->class)) {
    dev_err(&client->dev, "class_create failed\n");
    cdev_del(&mpu->cdev);
    unregister_chrdev_region(mpu->dev_num, 1);
    return PTR_ERR(mpu->class);
}

if (!device_create(mpu->class, NULL, mpu->dev_num, NULL, "mpu6050")) {
    dev_err(&client->dev, "device_create failed\n");
    class_destroy(mpu->class);
    cdev_del(&mpu->cdev);
    unregister_chrdev_region(mpu->dev_num, 1);
    return -ENODEV;
}

dev_info(&client->dev, "Created /dev/mpu6050\n");


    mpu->class = class_create(THIS_MODULE,"mpu6050");
    device_create(mpu->class, NULL, mpu->dev_num, NULL, "mpu6050");

    mpu->irq = client->irq;
    if (mpu->irq) {
        ret = request_threaded_irq(mpu->irq, NULL,
                                   mpu6050_irq_handler,
                                   IRQF_ONESHOT,
                                   "mpu6050_irq", mpu);
        if (ret)
            dev_warn(&client->dev, "IRQ request failed\n");
    }

    pr_info("MPU6050 probed\n");
    return 0;
}

static int mpu6050_remove(struct i2c_client *client)
{
    struct mpu6050_dev *mpu = i2c_get_clientdata(client);

    if (mpu->irq)
        free_irq(mpu->irq, mpu);

    device_destroy(mpu->class, mpu->dev_num);
    class_destroy(mpu->class);
    cdev_del(&mpu->cdev);
    unregister_chrdev_region(mpu->dev_num, 1);

    pr_info("MPU6050 removed\n");
    return 0;
}

/* ---------------- Tables ---------------- */

static const struct of_device_id mpu6050_of_match[] = {
    { .compatible = "SenseEdge,mpu6050" },
    { }
};
MODULE_DEVICE_TABLE(of, mpu6050_of_match);

static const struct i2c_device_id mpu6050_id[] = {
    { "mpu6050", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mpu6050_id);

static struct i2c_driver mpu6050_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = mpu6050_of_match,
    },
    .probe    = mpu6050_probe,
    .remove   = mpu6050_remove,
    .id_table = mpu6050_id,
};

module_i2c_driver(mpu6050_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Riddhi Sawarkar");
MODULE_DESCRIPTION("MPU6050 Embedded Linux Driver");
