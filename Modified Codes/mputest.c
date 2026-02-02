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

/* Sensitivity (for default ±2g, ±250 dps) */
#define ACCEL_SENS 16384.0
#define GYRO_SENS 131.0

struct mpu6050_data
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
};

int main(int argc, char *argv[])
{
    int fd;
    struct mpu6050_data data;
    ssize_t ret;

    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;

    int accel_range = 0x00; /* ±2g */
    int gyro_range = 0x00;  /* ±250 dps */

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

    /* Configure sensor via ioctl */
    if (ioctl(fd, MPU6050_SET_ACCEL_RANGE, &accel_range) < 0)
        perror("ioctl: set accel range");

    if (ioctl(fd, MPU6050_SET_GYRO_RANGE, &gyro_range) < 0)
        perror("ioctl: set gyro range");

    printf("MPU6050 configured, waiting for data...\n");

    while (1)
    {
        ret = read(fd, &data, sizeof(data));

        if (ret == sizeof(data))
        {

            ax_g = data.ax / ACCEL_SENS;
            ay_g = data.ay / ACCEL_SENS;
            az_g = data.az / ACCEL_SENS;

            gx_dps = data.gx / GYRO_SENS;
            gy_dps = data.gy / GYRO_SENS;
            gz_dps = data.gz / GYRO_SENS;

            printf("ACC[g]: X=%7.3f Y=%7.3f Z=%7.3f | "
                   "GYR[dps]: X=%7.3f Y=%7.3f Z=%7.3f\n",
                   ax_g, ay_g, az_g,
                   gx_dps, gy_dps, gz_dps);
        }
        else if (ret == 0)
        {
            printf("Device closed by driver\n");
            break;
        }
        else
        {
            if (errno == ENODEV)
            {
                printf("Device removed\n");
                break;
            }
            perror("read");
            break;
        }
    }

    close(fd);
    return 0;
}