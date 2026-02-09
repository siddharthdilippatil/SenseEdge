#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/kfifo.h>
#include <linux/mutex.h>

#define PCHAR_BUF_SIZE 256

/* ---------- Device Private Structure ---------- */
struct pchar_dev
{
    struct cdev cdev;
    struct kfifo buffer;
    struct mutex lock;

    /* Placeholder:
     * - Member 3 can attach interrupt state
     * - Member 2 can link sensor backend
     */
};

/* ---------- OPEN ---------- */
static int pchar_open(struct inode *inode, struct file *file)
{
    struct pchar_dev *dev;

    dev = container_of(inode->i_cdev, struct pchar_dev, cdev);
    file->private_data = dev;

    pr_info("pchar: device opened\n");
    return 0;
}

/* ---------- RELEASE ---------- */
static int pchar_release(struct inode *inode, struct file *file)
{
    pr_info("pchar: device closed\n");
    return 0;
}

/* ---------- READ ---------- */
static ssize_t pchar_read(struct file *file,
                          char __user *buf,
                          size_t count,
                          loff_t *ppos)
{
    struct pchar_dev *dev = file->private_data;
    unsigned int copied;
    int ret;

    if (kfifo_is_empty(&dev->buffer))
        return 0;

    mutex_lock(&dev->lock);

    ret = kfifo_to_user(&dev->buffer, buf, count, &copied);

    mutex_unlock(&dev->lock);

    if (ret)
        return ret;

    return copied;
}

/* ---------- WRITE ---------- */
static ssize_t pchar_write(struct file *file,
                           const char __user *buf,
                           size_t count,
                           loff_t *ppos)
{
    struct pchar_dev *dev = file->private_data;
    unsigned int copied;
    int ret;

    mutex_lock(&dev->lock);

    ret = kfifo_from_user(&dev->buffer, buf, count, &copied);

    mutex_unlock(&dev->lock);

    if (ret)
        return ret;

    return copied;
}

/* ---------- FILE OPERATIONS ---------- */
static const struct file_operations pchar_fops = {
    .owner = THIS_MODULE,
    .open = pchar_open,
    .release = pchar_release,
    .read = pchar_read,
    .write = pchar_write,
    .llseek = no_llseek,
};
