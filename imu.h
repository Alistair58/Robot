#ifndef IMU_H
#define IMU_H

#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

//Slave address of ICM-20948
#define IMU_I2C_ADDR 0x69
#define IMU_I2C_SDA_PIN 6
#define IMU_I2C_SCL_PIN 7

//The magnetometer is a separate chip (AK09916) and 
//this address allows pass-through access so it
#define AK09916_ADDR 0x0C 

typedef enum icm20948Banks{
    BANK_0 = 0x00,
    BANK_1 = 0x10,
    BANK_2 = 0x20,
    BANK_3 = 0x30,
}icm20948Banks;

typedef enum icm20948Bank0{
    WHOAMI = 0x00,
    USER_CTRL = 0x03,
    PWR_MGMT_1 = 0x06,
    INT_PIN_CFG = 0x0F,
    I2C_MST_STATUS = 0x17,
    ACCEL_XOUT_H = 0x2D,
    GYRO_XOUT_H = 0x33,
    EXT_SLV_SENS_DATA_00 = 0x3B,
    REG_BANK_SEL = 0x7F,
}icm20948Bank0;

typedef enum icm20948Bank2{
    GYRO_CONFIG_1 = 0x01,
    ACCEL_CONFIG = 0x14,
}icm20948Bank2;

typedef enum icm20948Bank3{
    I2C_MST_CTRL = 0x01,
    I2C_SLV0_ADDR  = 0x03,
    I2C_SLV0_REG = 0x04,
    I2C_SLV0_CTRL = 0x05,
    I2C_SLV0_DO = 0x06,
}icm20948Bank3;

typedef enum ak09916Registers{
    DEVICE_ID = 0x01,
    STATUS_1 = 0x10,
    MEASUREMENT_DATA = 0x11,
    CONTROL_2 = 0x31,
}ak09916Register;

void imu_i2c_init(void);
void mag_read(float mag_data[3]);
void gyro_read(float gyro_data[3]);
void accel_read(float accel_data[3]);
void calibrate_gyro_blocking(void);
void calibrate_mag_blocking(void);
void calibrate_accel_blocking(void);



#endif