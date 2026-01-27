#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

struct mpu6050_data {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
};

int main(void)
{
    int fd;
    struct mpu6050_data data;

    fd = open("/dev/mpu6050", O_RDONLY);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    while (1) {
        if (read(fd, &data, sizeof(data)) == sizeof(data)) {
            printf("ACC: %6d %6d %6d | GYR: %6d %6d %6d\n",
                   data.ax, data.ay, data.az,
                   data.gx, data.gy, data.gz);
        }
        sleep(1);   //1s delay
    }

    close(fd);
    return 0;
}

