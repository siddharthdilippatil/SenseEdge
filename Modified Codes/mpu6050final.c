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
#include <linux/idr.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/pm.h>

#define DRIVER_NAME "mpu6050"
#define MAX_DEVICES 8

/* MPU6050 Registers */
#define MPU6050_PWR_MGMT1 0x6B
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_ACCEL_XOUT 0x3B
#define MPU6050_ACCEL_CFG 0x1C
#define MPU6050_GYRO_CFG 0x1B
#define MPU6050_INT_ENABLE 0x38

/* ioctl commands */
#define MPU6050_SET_ACCEL_RANGE _IOW('m', 1, int)
#define MPU6050_SET_GYRO_RANGE _IOW('m', 2, int)

/* ---------- Global Char Framework ---------- */

static dev_t mpu6050_devt;
static struct class *mpu6050_class;
static DEFINE_IDA(mpu6050_ida);

/* ---------- Sensor Data ---------- */

struct mpu6050_data
{
    s16 ax, ay, az;
    s16 gx, gy, gz;
};

/* ---------- Per Device ---------- */

struct mpu6050_dev
{
    struct i2c_client *client;
    struct mutex lock;

    struct cdev cdev;
    dev_t devno;
    int minor;

    /* Cached config */
    u32 sampling_rate_hz;
    u32 accel_range_g;
    u32 gyro_range_dps;

    /* IRQ + waitqueue */
    int irq;
    bool data_ready;
    wait_queue_head_t wq;

    bool present;
};

/* ---------- I2C Helpers ---------- */

static int mpu6050_read_burst(struct i2c_client *client,
                              u8 reg, u8 *buf, int len)
{
    struct i2c_msg msgs[2] = {
        {client->addr, 0, 1, &reg},
        {client->addr, I2C_M_RD, len, buf}};
    return (i2c_transfer(client->adapter, msgs, 2) == 2) ? 0 : -EIO;
}

/* ---------- DT Parsing ---------- */

static void mpu6050_parse_dt(struct device *dev, struct mpu6050_dev *mpu)
{
    struct device_node *np = dev->of_node;

    mpu->sampling_rate_hz = 100;
    mpu->accel_range_g = 2;
    mpu->gyro_range_dps = 250;

    if (!np)
        return;

    of_property_read_u32(np, "sampling-rate-hz", &mpu->sampling_rate_hz);
    of_property_read_u32(np, "accel-range", &mpu->accel_range_g);
    of_property_read_u32(np, "gyro-range", &mpu->gyro_range_dps);
}

/* ---------- Apply Config ---------- */

static void mpu6050_apply_config(struct mpu6050_dev *mpu)
{
    u8 a = 0, g = 0;

    if (mpu->accel_range_g == 4)
        a = 0x08;
    else if (mpu->accel_range_g == 8)
        a = 0x10;
    else if (mpu->accel_range_g == 16)
        a = 0x18;

    if (mpu->gyro_range_dps == 500)
        g = 0x08;
    else if (mpu->gyro_range_dps == 1000)
        g = 0x10;
    else if (mpu->gyro_range_dps == 2000)
        g = 0x18;

    i2c_smbus_write_byte_data(mpu->client, MPU6050_ACCEL_CFG, a);
    i2c_smbus_write_byte_data(mpu->client, MPU6050_GYRO_CFG, g);
}

/* ---------- IRQ Handler ---------- */

static irqreturn_t mpu6050_irq_handler(int irq, void *dev_id)
{
    struct mpu6050_dev *mpu = dev_id;

    mpu->data_ready = true;
    wake_up_interruptible(&mpu->wq);

    return IRQ_HANDLED;
}

/* ---------- File Operations ---------- */

static int mpu6050_open(struct inode *inode, struct file *file)
{
    struct mpu6050_dev *mpu =
        container_of(inode->i_cdev, struct mpu6050_dev, cdev);

    if (!mpu->present)
        return -ENODEV;

    file->private_data = mpu;
    return 0;
}

static int mpu6050_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}

static ssize_t mpu6050_read(struct file *file,
                            char __user *buf,
                            size_t count, loff_t *ppos)
{
    struct mpu6050_dev *mpu = file->private_data;
    struct mpu6050_data data;
    u8 raw[14];

    if (!mpu || !mpu->present)
        return -ENODEV;

    wait_event_interruptible(mpu->wq, mpu->data_ready);

    mutex_lock(&mpu->lock);

    if (mpu6050_read_burst(mpu->client, MPU6050_ACCEL_XOUT, raw, 14))
    {
        mutex_unlock(&mpu->lock);
        return -EIO;
    }

    mpu->data_ready = false;

    data.ax = (raw[0] << 8) | raw[1];
    data.ay = (raw[2] << 8) | raw[3];
    data.az = (raw[4] << 8) | raw[5];
    data.gx = (raw[8] << 8) | raw[9];
    data.gy = (raw[10] << 8) | raw[11];
    data.gz = (raw[12] << 8) | raw[13];

    mutex_unlock(&mpu->lock);

    return copy_to_user(buf, &data, sizeof(data)) ? -EFAULT : sizeof(data);
}

static long mpu6050_ioctl(struct file *file,
                          unsigned int cmd, unsigned long arg)
{
    struct mpu6050_dev *mpu = file->private_data;
    int val;

    if (!mpu || !mpu->present)
        return -ENODEV;

    if (copy_from_user(&val, (int __user *)arg, sizeof(int)))
        return -EFAULT;

    mutex_lock(&mpu->lock);

    if (cmd == MPU6050_SET_ACCEL_RANGE)
        mpu->accel_range_g = val;
    else if (cmd == MPU6050_SET_GYRO_RANGE)
        mpu->gyro_range_dps = val;
    else
    {
        mutex_unlock(&mpu->lock);
        return -EINVAL;
    }

    mpu6050_apply_config(mpu);
    mutex_unlock(&mpu->lock);
    return 0;
}

static const struct file_operations mpu6050_fops = {
    .owner = THIS_MODULE,
    .open = mpu6050_open,
    .release = mpu6050_release,
    .read = mpu6050_read,
    .unlocked_ioctl = mpu6050_ioctl,
    .llseek = no_llseek,
};

/* ---------- sysfs ---------- */

#define SYSFS_RW(_name)                                                                        \
    static ssize_t _name##_show(struct device *dev,                                            \
                                struct device_attribute *attr, char *buf)                      \
    {                                                                                          \
        struct mpu6050_dev *mpu = dev_get_drvdata(dev);                                        \
        return sprintf(buf, "%u\n", mpu->_name);                                               \
    }                                                                                          \
    static ssize_t _name##_store(struct device *dev,                                           \
                                 struct device_attribute *attr, const char *buf, size_t count) \
    {                                                                                          \
        struct mpu6050_dev *mpu = dev_get_drvdata(dev);                                        \
        kstrtou32(buf, 10, &mpu->_name);                                                       \
        mutex_lock(&mpu->lock);                                                                \
        mpu6050_apply_config(mpu);                                                             \
        mutex_unlock(&mpu->lock);                                                              \
        return count;                                                                          \
    }                                                                                          \
    static DEVICE_ATTR_RW(_name)

SYSFS_RW(accel_range_g);
SYSFS_RW(gyro_range_dps);
SYSFS_RW(sampling_rate_hz);

/* ---------- PM ---------- */

static int mpu6050_suspend(struct device *dev)
{
    struct mpu6050_dev *mpu = dev_get_drvdata(dev);
    i2c_smbus_write_byte_data(mpu->client, MPU6050_PWR_MGMT1, 0x40);
    return 0;
}

static int mpu6050_resume(struct device *dev)
{
    struct mpu6050_dev *mpu = dev_get_drvdata(dev);
    i2c_smbus_write_byte_data(mpu->client, MPU6050_PWR_MGMT1, 0x00);
    mpu6050_apply_config(mpu);
    return 0;
}

static const struct dev_pm_ops mpu6050_pm_ops = {
    .suspend = mpu6050_suspend,
    .resume = mpu6050_resume,
};

/* ---------- Probe & Remove ---------- */

static int mpu6050_probe(struct i2c_client *client)
{
    struct mpu6050_dev *mpu;
    int ret;
    u8 whoami;

    mpu = kzalloc(sizeof(*mpu), GFP_KERNEL);
    if (!mpu)
        return -ENOMEM;

    mutex_init(&mpu->lock);
    init_waitqueue_head(&mpu->wq);

    mpu->client = client;
    mpu->present = true;
    mpu->data_ready = false;

    i2c_set_clientdata(client, mpu);

    i2c_smbus_write_byte_data(client, MPU6050_PWR_MGMT1, 0x00);
    msleep(10);

    whoami = i2c_smbus_read_byte_data(client, MPU6050_WHO_AM_I);
    if (whoami != 0x68)
    {
        ret = -ENODEV;
        goto err_free;
    }

    mpu6050_parse_dt(&client->dev, mpu);
    mpu6050_apply_config(mpu);

    mpu->irq = client->irq;
    if (mpu->irq > 0)
    {
        ret = request_irq(mpu->irq, mpu6050_irq_handler,
                          IRQF_TRIGGER_RISING,
                          DRIVER_NAME, mpu);
        if (ret)
            goto err_free;

        i2c_smbus_write_byte_data(client, MPU6050_INT_ENABLE, 0x01);
    }

    mpu->minor = ida_alloc(&mpu6050_ida, GFP_KERNEL);
    if (mpu->minor < 0)
    {
        ret = mpu->minor;
        goto err_irq;
    }

    mpu->devno = MKDEV(MAJOR(mpu6050_devt), mpu->minor);

    cdev_init(&mpu->cdev, &mpu6050_fops);
    ret = cdev_add(&mpu->cdev, mpu->devno, 1);
    if (ret)
        goto err_ida;

    device_create(mpu6050_class, &client->dev,
                  mpu->devno, mpu, "mpu6050%d", mpu->minor);

    device_create_file(&client->dev, &dev_attr_accel_range_g);
    device_create_file(&client->dev, &dev_attr_gyro_range_dps);
    device_create_file(&client->dev, &dev_attr_sampling_rate_hz);

    dev_info(&client->dev, "mpu6050%d registered\n", mpu->minor);
    return 0;

err_ida:
    ida_free(&mpu6050_ida, mpu->minor);
err_irq:
    if (mpu->irq > 0)
        free_irq(mpu->irq, mpu);
err_free:
    kfree(mpu);
    return ret;
}

static int mpu6050_remove(struct i2c_client *client)
{
    struct mpu6050_dev *mpu = i2c_get_clientdata(client);

    device_remove_file(&client->dev, &dev_attr_accel_range_g);
    device_remove_file(&client->dev, &dev_attr_gyro_range_dps);
    device_remove_file(&client->dev, &dev_attr_sampling_rate_hz);

    device_destroy(mpu6050_class, mpu->devno);
    cdev_del(&mpu->cdev);
    ida_free(&mpu6050_ida, mpu->minor);

    if (mpu->irq > 0)
        free_irq(mpu->irq, mpu);

    mpu->present = false;
    kfree(mpu);
    return 0;
}

/* ---------- DT Match ---------- */

static const struct of_device_id mpu6050_of_match[] = {
    {.compatible = "senseedge,mpu6050"},
    {}};
MODULE_DEVICE_TABLE(of, mpu6050_of_match);

/* ---------- I2C Driver ---------- */

static struct i2c_driver mpu6050_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = mpu6050_of_match,
        .pm = &mpu6050_pm_ops,
    },
    .probe_new = mpu6050_probe,
    .remove = mpu6050_remove,
};

/* ---------- Init / Exit ---------- */

static int __init mpu6050_init(void)
{
    int ret;

    ret = alloc_chrdev_region(&mpu6050_devt, 0, MAX_DEVICES, DRIVER_NAME);
    if (ret)
        return ret;

    mpu6050_class = class_create(THIS_MODULE, DRIVER_NAME);
    if (IS_ERR(mpu6050_class))
    {
        unregister_chrdev_region(mpu6050_devt, MAX_DEVICES);
        return PTR_ERR(mpu6050_class);
    }

    return i2c_add_driver(&mpu6050_driver);
}

static void __exit mpu6050_exit(void)
{
    i2c_del_driver(&mpu6050_driver);
    class_destroy(mpu6050_class);
    unregister_chrdev_region(mpu6050_devt, MAX_DEVICES);
}

module_init(mpu6050_init);
module_exit(mpu6050_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Riddhi Sawarkar");
MODULE_DESCRIPTION("MPU6050 Multi-Device DT + IRQ + sysfs + PM Linux Driver");