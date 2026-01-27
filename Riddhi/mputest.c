#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

#define ACCEL_SENS 16384.0   // LSB/g for ±2g
#define GYRO_SENS  131.0     // LSB/(°/s) for ±250 dps

struct mpu6050_data {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
};

int main(void)
{
    int fd;
    struct mpu6050_data data;

    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;

    fd = open("/dev/mpu6050", O_RDONLY);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    printf("MPU6050 opened\n");

    while (1) {
        if (read(fd, &data, sizeof(data)) == sizeof(data)) {

            /* Convert raw values */
            ax_g = data.ax / ACCEL_SENS;
            ay_g = data.ay / ACCEL_SENS;
            az_g = data.az / ACCEL_SENS;

            gx_dps = data.gx / GYRO_SENS;
            gy_dps = data.gy / GYRO_SENS;
            gz_dps = data.gz / GYRO_SENS;

            printf("ACC[g]: X=%7.3f m/sec.sq Y=%7.3f m/sec.sq Z=%7.3f m/sec.sq | "
                   "GYR[degrees per second]: X=%7.3f dps Y=%7.3f dps Z=%7.3f dps\n",
                   ax_g, ay_g, az_g,
                   gx_dps, gy_dps, gz_dps);
        }

        sleep(1);   // 1 second delay
    }

    close(fd);
    return 0;
}
