#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>

#include <MQTTClient.h>

/* ioctl commands  */
#define MPU6050_SET_ACCEL_RANGE _IOW('m', 1, int)
#define MPU6050_SET_GYRO_RANGE _IOW('m', 2, int)

/* MQTT configuration */
#define MQTT_ADDRESS "tcp://localhost:1883"
#define MQTT_CLIENTID "bbb_mpu6050"
#define MQTT_TOPIC "mpu6050/bbb/imu"
#define MQTT_QOS 0
#define MQTT_TIMEOUT 1000L

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
    char payload[256];

    MQTTClient client;
    MQTTClient_connectOptions conn_opts =
        MQTTClient_connectOptions_initializer;
    MQTTClient_message pubmsg =
        MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;

    if (argc != 2)
    {
        fprintf(stderr, "Usage: %s /dev/mpu6050X\n", argv[0]);
        return 1;
    }

    /* Open device */
    fd = open(argv[1], O_RDONLY);
    if (fd < 0)
    {
        perror("open");
        return 1;
    }

    printf("Opened %s\n", argv[1]);

    /* Configure sensor */
    ioctl(fd, MPU6050_SET_ACCEL_RANGE, &accel_range);
    ioctl(fd, MPU6050_SET_GYRO_RANGE, &gyro_range);

    a_sens = accel_sens(accel_range);
    g_sens = gyro_sens(gyro_range);

    printf("Configured: ±%dg accel, ±%d dps gyro\n",
           accel_range, gyro_range);

    /* MQTT setup */
    MQTTClient_create(&client, MQTT_ADDRESS,
                      MQTT_CLIENTID,
                      MQTTCLIENT_PERSISTENCE_NONE, NULL);

    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;

    if (MQTTClient_connect(client, &conn_opts) != MQTTCLIENT_SUCCESS)
    {
        fprintf(stderr, "MQTT connect failed\n");
        close(fd);
        return 1;
    }

    printf("Connected to MQTT broker\n");
    printf("Publishing to topic: %s\n", MQTT_TOPIC);
    printf("Waiting for sensor interrupts...\n");

    while (1)
    {
        ret = read(fd, &data, sizeof(data));

        if (ret == sizeof(data))
        {

            float ax = data.ax / a_sens;
            float ay = data.ay / a_sens;
            float az = data.az / a_sens;

            float gx = data.gx / g_sens;
            float gy = data.gy / g_sens;
            float gz = data.gz / g_sens;

            /* JSON payload */
            snprintf(payload, sizeof(payload),
                     "{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
                     "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f}",
                     ax, ay, az, gx, gy, gz);

            pubmsg.payload = payload;
            pubmsg.payloadlen = strlen(payload);
            pubmsg.qos = MQTT_QOS;
            pubmsg.retained = 0;

            MQTTClient_publishMessage(client, MQTT_TOPIC,
                                      &pubmsg, &token);
            MQTTClient_waitForCompletion(client,
                                         token, MQTT_TIMEOUT);

            printf("Published: %s\n", payload);
        }
        else if (ret < 0 && errno == EINTR)
        {
            continue;
        }
        else
        {
            if (errno == EIO)
            {
                /* transient I2C error – retry */
                continue;
            }
            perror("read");
            break;
        }
    }

    MQTTClient_disconnect(client, 1000);
    MQTTClient_destroy(&client);
    close(fd);

    return 0;
}
