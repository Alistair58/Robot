#include "imu.h"

i2c_inst_t *i2c = i2c1;

static void imu_reset(){
    printf("\nResetting");
    //Register 0x6B is PWR_MGMT_1
    //0 resets the unit
    uint8_t buffer[] = {0x6B, 0x00};
    i2c_write_blocking(i2c, IMU_I2C_ADDR,buffer,2,false);
    sleep_ms(10);

    printf("\nConfiguring gyro");
    //set sensitivities
    //GYRO_CONFIG - 0x1B
    //Set bits [4:3] to 01 for 500dps
    uint8_t gyroConfigBuff[] = {0x1B,0b00001000};
    i2c_write_blocking(i2c, IMU_I2C_ADDR,gyroConfigBuff,2,false);
    sleep_ms(10);


    printf("\nConfiguring accel");
    //ACCEL_CONFIG - 0x1C
    //Set bits [4:3] to 00 for 2g
    uint8_t accConfigBuff[] = {0x1C,0b00000000};
    i2c_write_blocking(i2c, IMU_I2C_ADDR,accConfigBuff,2,false);
    sleep_ms(10);
}

static void imu_mag_read(uint16_t magData[3]){
    //Tell it to access magnetometer
    //Read from magnetometer

    //Slave 0 control
    //The MPU-9250 communicates with its slave the magnetometer

    //I2C_MST_CTRL 0x24
    //Enable - MSB is 1
    //LSBs are clock settings - I've just gone with 0

    //I2C_SLV0_ADDR 0x25
    //Reading - MSB is 1
    //From address 0x0C which is the magnetometer - LSB is 0x0C

    //I2C_SLV0_REG (0x26)
    //Address of where to begin data transfer - Magnetometer values
    
    //I2C_SLV0_EN (0x27)
    //Enable reading data from the slave - MSB is 1 (0x80)
    //The registers start at odd numbers set bit 4 to be 1
    //Reading 6 bytes (2 bytes * x,y,z) - LSB is 6

    uint8_t buffer[] = {0x24,0x80,0x0C|0x80,IMU_MAG_VALUES_ADDR,0x80|0b10000|0x6};
    i2c_write_blocking(i2c,IMU_I2C_ADDR,buffer,5,false);

    //It will start storing data at EXT_SENS_DATA_00
    sleep_ms(10);
    //Tell it that we want to access EXT_SENS_DATA_00
    //We give it the register with a 1 at the MSB
    //All registers are 7 bits long and a 1 MSB means read and a 0 MSB means write
    uint8_t buffer2[] = {0x49|0x80};
    i2c_write_blocking(i2c,IMU_I2C_ADDR,buffer2,1,false);

    //The IMU then puts it in the FIFO and we read it
    sleep_ms(10);
    uint8_t magDataRaw[6] = {0};
    i2c_read_blocking(i2c,IMU_I2C_ADDR,magDataRaw,6,false);

    for (int i=0;i<3;i++) {
        magData[i] = (magDataRaw[i*2] << 8 | magDataRaw[(i*2)+1]);
    }
    printf("\nMagnetometer - X: %d Y: %d Z: %d",magData[0],magData[1],magData[2]);

    //Disable slave?
}

static void imu_gyro_read(uint16_t gyroData[3]){
    uint8_t registerSelectionBuff[] = {0x43|0x80};
    i2c_write_blocking(i2c,IMU_I2C_ADDR,registerSelectionBuff,1,false);
    sleep_ms(10);
    uint8_t gyroDataRaw[6];
    i2c_read_blocking(i2c,IMU_I2C_ADDR,gyroDataRaw,6,false);
    for (int i=0;i<3;i++) {
        gyroData[i] = (gyroDataRaw[i*2] << 8 | gyroDataRaw[(i*2)+1]);
    }
    printf("\nGyroscope - X: %d Y: %d Z: %d",gyroData[0],gyroData[1],gyroData[2]);
    sleep_ms(10);
}

void imu_i2c_init(void){
    //TODO
    //remove
    sleep_ms(2000);
    i2c_init(i2c,400*1000); //400kHz
    gpio_set_function(IMU_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_I2C_SDA_PIN);
    gpio_pull_up(IMU_I2C_SCL_PIN);


    uint8_t whoAmIBuff[] = {0x75|0x80};
    int ret = i2c_write_blocking(i2c,IMU_I2C_ADDR,whoAmIBuff,1,false);
    printf("Ret: %d",ret);
    if(ret<=0){
        printf("\nCould not communicate with IMU");
        return;
    }
    sleep_ms(10);
    uint8_t whoAmIRes[1] = {0};
    i2c_read_blocking(i2c,IMU_I2C_ADDR,whoAmIRes,1,false);
    printf("\nwhoAmIRes[0]: %x",whoAmIRes[0]);
    if(whoAmIRes[0]==0x70){ //documentation says 0x71 but mine seems to be 0x70
        printf("\nSuccessfully made contact with IMU");
    }
    else{
        printf("\nCould not communicate with IMU.");
        return;
    }

    imu_reset();

    while(1){
        uint16_t magVals[3] = {0};
        imu_mag_read(magVals);
        uint16_t gyroVals[3] = {0};
        imu_gyro_read(gyroVals);
        sleep_ms(1000);
    }

    
}