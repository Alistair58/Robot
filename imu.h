#ifndef IMU_H
#define IMU_H

//Slave address of MPU-9250
#define IMU_I2C_ADDR 0x68
#define IMU_I2C_SDA_PIN 6
#define IMU_I2C_SCL_PIN 7

//The magnetometer is a separate chip (AK8963) and 
//this address allows pass-through access so it
#define IMU_MAG_ADDR 0x0C 

//Continguous 2 byte values x,y,z
#define IMU_MAG_VALUES_ADDR 0x03

#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"


void imu_i2c_init(void);
static void imu_reset();
#endif