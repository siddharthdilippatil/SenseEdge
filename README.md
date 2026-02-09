# SenseEdge

Smart Motion Sensing Node using Custom Linux Device Driver and IoT Integration

This project implements a complete motion sensing system using the **MPU6050 IMU sensor**, developed and validated across both **bare-metal microcontroller firmware** and **Embedded Linux** environments. The objective was to design a production-grade sensor stack that demonstrates low-level hardware understanding, Linux driver development, and real-time data visualization.

The development began with **bare-metal firmware on an STM32 microcontroller**, where the MPU6050 was interfaced using register-level I2C communication. This stage focused on validating the sensor configuration sequence, register map, scaling factors, and raw data correctness without any operating system abstraction. Accelerometer and gyroscope outputs were verified on a character LCD, ensuring that the sensor behaved as expected before migrating to Linux. This step reduced integration risk and established confidence in the hardware and protocol logic.

After validation on STM32, the system was ported to the **BeagleBone Black (AM335x SoC)** running Embedded Linux. A **custom Linux kernel I2C device driver** was developed for the MPU6050 using the Linux I2C subsystem. The driver supports proper **probe and remove callbacks**, **Device Tree–based hardware binding**, and **runtime configuration**. Sensor data acquisition is implemented using an **interrupt-driven model**, where the MPU6050 data-ready interrupt wakes a kernel wait queue, enabling efficient, low-latency reads without polling.

The driver exposes sensor data to user space through a **pseudo character device**, supporting multiple device instances. Configuration parameters such as accelerometer and gyroscope ranges can be modified at runtime using **ioctl system calls** and **sysfs attributes**, demonstrating integration with the Linux device model. Basic **power management support** is implemented using suspend and resume hooks, allowing the sensor to enter low-power sleep states and restore configuration on wake-up also the driver is registered using a **device tree overlay** file.

On the user-space side, a C application reads real-time motion data from the character device and converts raw values into physical units. For visualization and system-level integration, the processed data is published using **MQTT**, with the BeagleBone Black acting as the data source. **Node-RED** is used to subscribe to the MQTT topic and visualize orientation using a 3D model, enabling real-time motion tracking without cloud dependency.

**Contributors:**
Siddharth Patil, Riddhi Sawarkar, Mrunal Deshpande, Harsh Randive
