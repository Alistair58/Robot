#include "imu.h"

static int icm20948_read(uint8_t reg,uint8_t *buffer,int len);
static int icm20948_write(uint8_t *buffer,int len);
static bool icm20948_init();
static void icm20948_bypass_scan();
static void mag_read(int16_t magData[3]);
static void gyro_read(int16_t gyroData[3]);
static int icm20948_write_byte(uint8_t reg,uint8_t byte);
i2c_inst_t *i2c = i2c1;


static int icm20948_read(uint8_t reg,uint8_t *buffer,int len){
    int bytes_written = i2c_write_blocking(i2c,IMU_I2C_ADDR,&reg,1,false);
    if(bytes_written<=0){
        printf("\nCould not communicate with IMU. Error code: %d",bytes_written);
        return bytes_written;
    }
    //The IMU then puts it in the FIFO and we read it
    sleep_ms(10);
    bytes_written = i2c_read_blocking(i2c,IMU_I2C_ADDR,buffer,len,false);
    if(bytes_written<=0){
        printf("\nCould not communicate with IMU. Error code: %d",bytes_written);
    }
    return bytes_written;
}

static int icm20948_write(uint8_t *buffer,int len){
    int bytes_written = i2c_write_blocking(i2c, IMU_I2C_ADDR,buffer,len,false);
    if(bytes_written<=0){
        printf("\nCould not communicate with IMU. Error code: %d",bytes_written);
        return bytes_written;
    }
    sleep_ms(10);
    return bytes_written;
}

static int icm20948_write_byte(uint8_t reg,uint8_t byte){
    uint8_t buffer[] = {reg,byte};
    int bytes_written = i2c_write_blocking(i2c, IMU_I2C_ADDR,buffer,2,false);
    if(bytes_written<=0){
        printf("\nCould not communicate with IMU. Error code: %d",bytes_written);
        return bytes_written;
    }
    sleep_ms(10);
    return bytes_written;
}

static bool icm20948_init(){
    icm20948_write_byte(REG_BANK_SEL,BANK_0);

    uint8_t whoami_res = 0x00;
    icm20948_read(WHOAMI,&whoami_res,1);

    printf("\nWhoami: 0x%x",whoami_res);
    const uint8_t ICM20948_WHOAMI = 0xEA;
    //Mine seems to be 0x70
    if(whoami_res==ICM20948_WHOAMI){ 
        printf("\nSuccessfully made contact with IMU");
    }
    else{
        printf("\nCould not communicate with IMU.");
        return false;
    }
    //MSB to 1 resets the unit
    const uint8_t PWR_MGMT_RESET = 0x80;
    icm20948_write_byte(PWR_MGMT_1,PWR_MGMT_RESET);
    //Datasheet says to wait
    sleep_ms(100);

    //Reset puts it into sleep
    //We don't want sleep
    const uint8_t PWR_MGMT_CLEAR_SLEEP = 0x01;
    icm20948_write_byte(PWR_MGMT_1,PWR_MGMT_CLEAR_SLEEP);

    //Set gyro sensitivity
    icm20948_write_byte(REG_BANK_SEL,BANK_2);

    //Enable low pass filter with -3DB BW at 23.9Hz and scale at +-500dps
    const uint8_t GYRO_CONFIG_VAL = 0b00100011;
    icm20948_write_byte(GYRO_CONFIG_1,GYRO_CONFIG_VAL);
    

    //Tell it to access magnetometer

    //Slave 0 control
    //The ICM-20948 communicates with its slave the magnetometer
    icm20948_write_byte(REG_BANK_SEL,BANK_0);
    //Disable I2C bypass mode
    const uint8_t INT_PIN_CFG_VAL = 0x00;
    icm20948_write_byte(INT_PIN_CFG,INT_PIN_CFG_VAL); 


    //Enable the ICM_20948 being the master to the magnetometer
    const uint8_t USER_CTRL_MASTER_ENABLE = 0x20;
    icm20948_write_byte(USER_CTRL,USER_CTRL_MASTER_ENABLE);

    icm20948_write_byte(REG_BANK_SEL,BANK_3);
   
    //Set clock speed to 400kHz
    const uint8_t I2C_MST_CTRL_CLK = 0x07; 
    icm20948_write_byte(I2C_MST_CTRL,I2C_MST_CTRL_CLK);

    //Reading - MSB is 1
    //From address 0x0C which is the magnetometer - LSB is 0x0C
    const uint8_t I2C_SLV0_ADDR_VAL = 0x80|AK09916_ADDR;
    icm20948_write_byte(I2C_SLV0_ADDR,I2C_SLV0_ADDR_VAL);

    const uint8_t I2C_SLV0_REG_VAL = DEVICE_ID; //Device ID register
    icm20948_write_byte(I2C_SLV0_REG,I2C_SLV0_REG_VAL);

    //Enable reading data from the slave - MSB is 1 (0x80)
    //The registers start at odd numbers set bit 4 to be 1
    //Reading 1 byte - LSB is 1
    const uint8_t I2C_SLV0_CTRL_VAL = 0x80|0x01;
    icm20948_write_byte(I2C_SLV0_CTRL,I2C_SLV0_CTRL_VAL);
    
    sleep_ms(100);
    icm20948_write_byte(REG_BANK_SEL,BANK_0);
    uint8_t mag_whoami_result = 0x00;
    icm20948_read(EXT_SLV_SENS_DATA_00,&mag_whoami_result,1);
    printf("\nAK09916 (Magnetometer chip) Whoami: %x",mag_whoami_result);

    const uint8_t AK09916_WHOAMI = 0x09;
    return mag_whoami_result==AK09916_WHOAMI;
}




static void icm20948_bypass_scan(){
    printf("\nEnabling I2C bypass mode");
    //Enable I2C bypass mode
    const uint8_t INT_PIN_CFG_ADDR = 0x37;
    const uint8_t INT_PIN_CFG_VAL = 0x02;
    icm20948_write_byte(INT_PIN_CFG_ADDR,INT_PIN_CFG_VAL); 
    for (uint8_t addr=3;addr<0x78;addr++) {
        // Just try to write 0 bytes — this is a common way to detect presence
        uint8_t dummy = 0;
        int result = i2c_write_blocking(i2c, addr, &dummy, 1, false);
        if (result>0) {
            printf("\nDevice found at 0x%02X. Bytes written: %d", addr,result);
        }
    }
    printf("\nI2C bypass mode ending. Only the devices listed above were found.");
}

static void mag_read(int16_t magData[3]){
    //Tell it to access magnetometer
    //Read from magnetometer

    //Slave 0 control
    //The ICM-20948 communicates with its slave the magnetometer

    //Disable I2C bypass mode
    const uint8_t INT_PIN_CFG_ADDR = 0x37;
    const uint8_t INT_PIN_CFG_VAL = 0x00;
    icm20948_write_byte(INT_PIN_CFG_ADDR,INT_PIN_CFG_VAL); 


    const uint8_t USER_CTRL_ADDR = 0x6A;
    //Enable the ICM-20948 being the master to the magnetometer
    const uint8_t USER_CTRL_MASTER_ENABLE = 0x20;
    icm20948_write_byte(USER_CTRL_ADDR,USER_CTRL_MASTER_ENABLE);

   
    const uint8_t I2C_MST_CTRL_ADDR = 0x24;
    //Set clock speed to 400kHz
    const uint8_t I2C_MST_CTRL_CLK = 0x0D; 
    icm20948_write_byte(I2C_MST_CTRL_ADDR,I2C_MST_CTRL_CLK);

    const uint8_t I2C_SLV0_ADDR_ADDR  = 0x25;
    //Reading - MSB is 1
    //From address 0x0C which is the magnetometer - LSB is 0x0C
    const uint8_t I2C_SLV0_ADDR_VAL = 0x80|AK09916_ADDR;
    icm20948_write_byte(I2C_SLV0_ADDR_ADDR,I2C_SLV0_ADDR_VAL);

    const uint8_t I2C_SLV0_REG_ADDR = 0x26;
    //Address of where to begin data transfer - Magnetometer values
    const uint8_t I2C_SLV0_REG_VAL = 0x03; //Measurement data register
    icm20948_write_byte(I2C_SLV0_REG_ADDR,I2C_SLV0_REG_VAL);

    const uint8_t I2C_SLV0_CTRL_ADDR = 0x27;
    //Enable reading data from the slave - MSB is 1 (0x80)
    //The registers start at odd numbers set bit 4 to be 1
    //Reading 6 bytes (x,y,z 2 each) - LSB is 6
    const uint8_t I2C_SLV0_CTRL_VAL = 0x80|0b00010000|0x06;
    icm20948_write_byte(I2C_SLV0_CTRL_ADDR,I2C_SLV0_CTRL_VAL);
    

    //It will start storing data at EXT_SENS_DATA_00
    sleep_ms(100);
    //Tell it that we want to access EXT_SENS_DATA_00
    //We give it the register with a 1 at the MSB
    //All registers are 7 bits long and a 1 MSB means read and a 0 MSB means write
    uint8_t magDataRaw[6] = {0};
    const uint8_t EXT_SENS_DATA_00_ADDR = 0x49;
    //The IMU then puts it in the FIFO and we read it
    icm20948_read(EXT_SENS_DATA_00_ADDR,magDataRaw,6);

    for (int i=0;i<3;i++) {
        magData[i] = ((int16_t) magDataRaw[i*2] << 8 | magDataRaw[(i*2)+1]);
    }
    printf("\nMagnetometer - X: %d Y: %d Z: %d",magData[0],magData[1],magData[2]);

}

static void gyro_read(int16_t gyroData[3]){
    const uint8_t GYRO_XOUT_H_ADDR = 0x43;
    uint8_t gyroDataRaw[6] = {0};
    icm20948_read(GYRO_XOUT_H_ADDR,gyroDataRaw,6);
    for (int i=0;i<3;i++) {
        gyroData[i] = ((int16_t)gyroDataRaw[i*2] << 8 | gyroDataRaw[(i*2)+1]);
    }
    printf("\nGyroscope - X: %d Y: %d Z: %d",gyroData[0],gyroData[1],gyroData[2]);
}

void imu_i2c_init(void){
    //TODO
    //remove sleep
    sleep_ms(5000);
    i2c_init(i2c,400*1000); //400kHz
    gpio_set_function(IMU_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_I2C_SDA_PIN);
    gpio_pull_up(IMU_I2C_SCL_PIN);

    bool connected = icm20948_init();
    if(!connected) return;

    // while(1){
    //     int16_t gyroVals[3] = {0};
    //     gyro_read(gyroVals);
    //     int16_t magVals[3] = {0};
    //     mag_read(magVals);
    //     sleep_ms(2000);
    // }

    
}