#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>

/* ioctl commands (must match driver) */
#define MPU6050_SET_ACCEL_RANGE _IOW('m', 1, int)
#define MPU6050_SET_GYRO_RANGE _IOW('m', 2, int)

struct mpu6050_data
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
};

/* Sensitivity helpers */
static float accel_sens(int g)
{
    switch (g)
    {
    case 4:
        return 8192.0;
    case 8:
        return 4096.0;
    case 16:
        return 2048.0;
    default:
        return 16384.0; /* 2g */
    }
}

static float gyro_sens(int dps)
{
    switch (dps)
    {
    case 500:
        return 65.5;
    case 1000:
        return 32.8;
    case 2000:
        return 16.4;
    default:
        return 131.0; /* 250 dps */
    }
}

int main(int argc, char *argv[])
{
    int fd;
    struct mpu6050_data data;
    ssize_t ret;

    int accel_range = 2;  /* ±2g */
    int gyro_range = 250; /* ±250 dps */

    float a_sens, g_sens;

    if (argc != 2)
    {
        fprintf(stderr, "Usage: %s /dev/mpu6050X\n", argv[0]);
        return 1;
    }

    fd = open(argv[1], O_RDONLY);
    if (fd < 0)
    {
        perror("open");
        return 1;
    }

    printf("Opened %s\n", argv[1]);

    /* Configure sensor */
    if (ioctl(fd, MPU6050_SET_ACCEL_RANGE, &accel_range) < 0)
        perror("ioctl accel");

    if (ioctl(fd, MPU6050_SET_GYRO_RANGE, &gyro_range) < 0)
        perror("ioctl gyro");

    a_sens = accel_sens(accel_range);
    g_sens = gyro_sens(gyro_range);

    printf("Configured: ±%dg accel, ±%d dps gyro\n",
           accel_range, gyro_range);

    printf("Waiting for sensor interrupts...\n");

    while (1)
    {
        ret = read(fd, &data, sizeof(data));

        if (ret == sizeof(data))
        {

            printf("ACC[g]: X=%7.3f Y=%7.3f Z=%7.3f | "
                   "GYR[dps]: X=%7.3f Y=%7.3f Z=%7.3f\n",
                   data.ax / a_sens,
                   data.ay / a_sens,
                   data.az / a_sens,
                   data.gx / g_sens,
                   data.gy / g_sens,
                   data.gz / g_sens);
        }
        else if (ret < 0 && errno == EINTR)
        {
            /* Interrupted by signal – retry */
            continue;
        }
        else if (ret == 0)
        {
            printf("Device closed\n");
            break;
        }
        else
        {
            perror("read");
            break;
        }
    }

    close(fd);
    return 0;
}