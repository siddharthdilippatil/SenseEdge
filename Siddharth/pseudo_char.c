#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/idr.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "mpu6050"
#define MAX_DEVICES 8

/* ================= GLOBAL CHAR FRAMEWORK ================= */

static dev_t mpu_devt;
static struct class *mpu_class;
static DEFINE_IDA(mpu_ida);

/* ================= PER-DEVICE STRUCTURE ================= */

struct mpu6050_pchar
{
    struct cdev cdev;
    dev_t devno;
    int minor;

    struct mutex lock;

    /* Link to actual device (set in probe by Member 3) */
    void *hw_priv;

    bool present;
};

/* ================= FILE OPERATIONS ================= */

static int mpu_open(struct inode *inode, struct file *file)
{
    struct mpu6050_pchar *pdev;

    pdev = container_of(inode->i_cdev,
                        struct mpu6050_pchar, cdev);

    if (!pdev->present)
        return -ENODEV;

    file->private_data = pdev;
    pr_info("mpu6050: device opened (minor=%d)\n", pdev->minor);
    return 0;
}

static int mpu_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    pr_info("mpu6050: device closed\n");
    return 0;
}

/*
 * NOTE:
 * Actual read/write/ioctl logic is implemented
 * by MEMBER 3 and MEMBER 1.
 * This layer only routes calls.
 */

static ssize_t mpu_read(struct file *file,
                        char __user *buf,
                        size_t count, loff_t *ppos)
{
    /* ---- HANDLED BY MEMBER 3 ---- */
    return -EINVAL;
}

static long mpu_ioctl(struct file *file,
                      unsigned int cmd, unsigned long arg)
{
    /* ---- HANDLED BY MEMBER 3 ---- */
    return -EINVAL;
}

static const struct file_operations mpu_fops = {
    .owner = THIS_MODULE,
    .open = mpu_open,
    .release = mpu_release,
    .read = mpu_read,
    .unlocked_ioctl = mpu_ioctl,
    .llseek = no_llseek,
};

/* ================= DEVICE CREATION HELPERS ================= */

struct mpu6050_pchar *mpu_pchar_create(struct device *parent)
{
    struct mpu6050_pchar *pdev;
    int ret;

    pdev = kzalloc(sizeof(*pdev), GFP_KERNEL);
    if (!pdev)
        return NULL;

    mutex_init(&pdev->lock);
    pdev->present = true;

    pdev->minor = ida_alloc(&mpu_ida, GFP_KERNEL);
    if (pdev->minor < 0)
        goto err_free;

    pdev->devno = MKDEV(MAJOR(mpu_devt), pdev->minor);

    cdev_init(&pdev->cdev, &mpu_fops);
    ret = cdev_add(&pdev->cdev, pdev->devno, 1);
    if (ret)
        goto err_ida;

    device_create(mpu_class, parent,
                  pdev->devno, NULL,
                  "mpu6050%d", pdev->minor);

    pr_info("mpu6050: pseudo char device created (/dev/mpu6050%d)\n",
            pdev->minor);
    return pdev;

err_ida:
    ida_free(&mpu_ida, pdev->minor);
err_free:
    kfree(pdev);
    return NULL;
}

void mpu_pchar_destroy(struct mpu6050_pchar *pdev)
{
    device_destroy(mpu_class, pdev->devno);
    cdev_del(&pdev->cdev);
    ida_free(&mpu_ida, pdev->minor);
note:
    pdev->present = false;
    kfree(pdev);

    pr_info("mpu6050: pseudo char device removed\n");
}

/* ================= MODULE INIT / EXIT ================= */

static int __init mpu_pchar_init(void)
{
    int ret;

    pr_info("mpu6050: pseudo char init\n");

    ret = alloc_chrdev_region(&mpu_devt, 0,
                              MAX_DEVICES, DRIVER_NAME);
    if (ret)
        return ret;

    mpu_class = class_create(THIS_MODULE, DRIVER_NAME);
    if (IS_ERR(mpu_class))
    {
        unregister_chrdev_region(mpu_devt, MAX_DEVICES);
        return PTR_ERR(mpu_class);
    }

    return 0;
}

static void __exit mpu_pchar_exit(void)
{
    class_destroy(mpu_class);
    unregister_chrdev_region(mpu_devt, MAX_DEVICES);
    pr_info("mpu6050: pseudo char exit\n");
}

module_init(mpu_pchar_init);
module_exit(mpu_pchar_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Siddharth Patil");
MODULE_DESCRIPTION("MPU6050 Pseudo Character Driver");
