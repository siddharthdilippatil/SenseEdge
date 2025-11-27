### mpu6050.h

    1. write slave address for read and write
        MPU6050 slave address = value of WHO_AM_I+AD0 pin
        Default= 0x68(WHO_AM_I PoR and AD0 to GND)(0110 1000)

        Read addr = 0xD1 (1101 0001)
        Write addr = 0xD0 (1101 0000)

    2. write addresses of significant registers
    3. ``` C
        #define WHO_AM_I 0x75
        #define PWR_MGMT_1 0x6B
        #define SMPLRT_DIV 0x19
        #define GYRO_CONFIG 0x1B
        #define ACCEL_CONFIG 0x1C
        #define ACCEL_XOUT_H 0x3B
        ````
    4. define a structure to store raw values
    5. define init and burst read functions

Note: only accel xout h register address is defined because of the burst read behaviour the subsequent readings from registers are given by mpu automatically as master sends ack hence no need to specify regerister addresses again

### mpu6050.c

1. mpu6050_init
   1.1 read who am i register of mpu 6050 to check mpu is accessible.
   (start->send slave addr with write bit->send regr addr->repeat start->send slave addr with read bit->read with no ack)

   1.2 check the value of read function if it is 0x68 if it is continue init.

   1.3 write 0x00 to PWR_MGMT_1 register to power on mpu (its in sleep mode by default)

   1.4 write 0x07 to SMPLRT_DIV register to set sample rate to 1khz (sample rate= (gyro o/p(8khz))/(1+ smplrt_div))

   1.5 write 0x00 to ACCEL_CONFIG and GYRO_CONFIG registers to set full scale values of +-2g and +-250degree respectively

   1.6 start reading from accel_xout_h keep reading, after each 2 readings i.e high and low enter the values in structure using the recieved structure pointer(note: the values are to be read in order accel x,y,z temp gyro x,y,z)

### main.c

    -add all header files
    -call all init functions
    -make a structure variable to store data into
    -in a while loop call burst read function and print values on lcd with necessary delays
