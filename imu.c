#include "imu.h"

static void imu_reset();
static int mpu9250_read(uint8_t reg,uint8_t *buffer,int len);
static int mpu9250_write(uint8_t *buffer,int len);
static bool mpu9250_test();
static bool imu_mag_test();
static void imu_mag_bypass_scan();
static void imu_gyro_read(uint16_t gyroData[3]);
static int mpu9250_write_byte(uint8_t reg,uint8_t byte);
i2c_inst_t *i2c = i2c1;

static void imu_reset(){
    printf("\nResetting");
    //Register 0x6B is PWR_MGMT_1
    //0 resets the unit
    const uint8_t PWR_MGMT_1_ADDR = 0x6B;
    const uint8_t PWR_MGMT_1_RESET = 0x00;
    uint8_t reset_cmd[] = {PWR_MGMT_1_ADDR, PWR_MGMT_1_RESET};
    mpu9250_write(reset_cmd,2);

    printf("\nConfiguring gyro");
    //set sensitivities
    //GYRO_CONFIG - 0x1B
    //Set bits [4:3] to 01 for 500dps
    const uint8_t GYRO_CONFIG_ADDR = 0x1B;
    const uint8_t GYRO_CONFIG_500DPS = 0b00001000;
    uint8_t gyro_config_cmd[] = {GYRO_CONFIG_ADDR,GYRO_CONFIG_500DPS};
    mpu9250_write(gyro_config_cmd,2);


    printf("\nConfiguring accel");
    //ACCEL_CONFIG - 0x1C
    //Set bits [4:3] to 00 for 2g
    const uint8_t ACCEL_CONFIG_ADDR = 0x1C;
    const uint8_t ACCEL_CONFIG_2G = 0b00000000;
    uint8_t accel_config_cmd[] = {ACCEL_CONFIG_ADDR,ACCEL_CONFIG_2G};
    mpu9250_write(accel_config_cmd,2);
}

static int mpu9250_read(uint8_t reg,uint8_t *buffer,int len){
    const uint8_t READ_BIT = 0x80;
    reg|=READ_BIT;
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

static int mpu9250_write(uint8_t *buffer,int len){
    int bytes_written = i2c_write_blocking(i2c, IMU_I2C_ADDR,buffer,len,false);
    if(bytes_written<=0){
        printf("\nCould not communicate with IMU. Error code: %d",bytes_written);
        return bytes_written;
    }
    sleep_ms(10);
    return bytes_written;
}

static int mpu9250_write_byte(uint8_t reg,uint8_t byte){
    uint8_t buffer[] = {reg,byte};
    int bytes_written = i2c_write_blocking(i2c, IMU_I2C_ADDR,buffer,2,false);
    if(bytes_written<=0){
        printf("\nCould not communicate with IMU. Error code: %d",bytes_written);
        return bytes_written;
    }
    sleep_ms(10);
    return bytes_written;
}

static bool mpu9250_test(){
    const uint8_t WHOAMI_ADDR = 0x75;
    uint8_t whoami_res = 0x00;
    mpu9250_read(WHOAMI_ADDR,&whoami_res,1);

    printf("\nWhoami: %x",whoami_res);
    const uint8_t MPU9250_WHOAMI = 0x71;
    const uint8_t MPU6500_WHOAMI = 0x70;
    //Mine seems to be 0x70
    if(whoami_res==MPU9250_WHOAMI || whoami_res==MPU6500_WHOAMI){ 
        printf("\nSuccessfully made contact with IMU");
        return true;
    }
    else{
        printf("\nCould not communicate with IMU.");
        return false;
    }
}


static bool imu_mag_test(){
    //Tell it to access magnetometer
    //Read from magnetometer

    //Slave 0 control
    //The MPU-9250 communicates with its slave the magnetometer

    //Disable I2C bypass mode
    const uint8_t INT_PIN_CFG_ADDR = 0x37;
    const uint8_t INT_PIN_CFG_VAL = 0x00;
    mpu9250_write_byte(INT_PIN_CFG_ADDR,INT_PIN_CFG_VAL); 


    const uint8_t USER_CTRL_ADDR = 0x6A;
    //Enable the MPU-9250 being the master to the magnetometer
    const uint8_t USER_CTRL_MASTER_ENABLE = 0x20;
    mpu9250_write_byte(USER_CTRL_ADDR,USER_CTRL_MASTER_ENABLE);

   
    const uint8_t I2C_MST_CTRL_ADDR = 0x24;
    //Set clock speed to 400kHz
    const uint8_t I2C_MST_CTRL_CLK = 0x0D; 
    mpu9250_write_byte(I2C_MST_CTRL_ADDR,I2C_MST_CTRL_CLK);

    const uint8_t I2C_SLV0_ADDR_ADDR  = 0x25;
    //Reading - MSB is 1
    //From address 0x0C which is the magnetometer - LSB is 0x0C
    const uint8_t I2C_SLV0_ADDR_VAL = 0x80|IMU_MAG_ADDR;
    mpu9250_write_byte(I2C_SLV0_ADDR_ADDR,I2C_SLV0_ADDR_VAL);

    const uint8_t I2C_SLV0_REG_ADDR = 0x26;
    //Address of where to begin data transfer - Magnetometer values
    const uint8_t I2C_SLV0_REG_VAL = 0x00; //Device ID register
    mpu9250_write_byte(I2C_SLV0_REG_ADDR,I2C_SLV0_REG_VAL);

    const uint8_t I2C_SLV0_CTRL_ADDR = 0x27;
    //Enable reading data from the slave - MSB is 1 (0x80)
    //The registers start at odd numbers set bit 4 to be 1
    //Reading 1 byte - LSB is 1
    const uint8_t I2C_SLV0_CTRL_VAL = 0x80|0x01;
    mpu9250_write_byte(I2C_SLV0_CTRL_ADDR,I2C_SLV0_CTRL_VAL);
    
    sleep_ms(100);
    const uint8_t EXT_SENS_DATA_00_ADDR = 0x49;
    uint8_t result = 0x00;
    mpu9250_read(EXT_SENS_DATA_00_ADDR,&result,1);
    printf("\nAK8963 Whoami: %x",result);
    return result==0x48;
}

static void imu_mag_bypass_scan(){
    //Enable I2C bypass mode
    const uint8_t INT_PIN_CFG_ADDR = 0x37;
    const uint8_t INT_PIN_CFG_VAL = 0x02;
    mpu9250_write_byte(INT_PIN_CFG_ADDR,INT_PIN_CFG_VAL); 
    for (uint8_t addr=3;addr<0x78;addr++) {
        // Just try to write 0 bytes — this is a common way to detect presence
        uint8_t dummy = 0;
        int result = i2c_write_blocking(i2c, addr, &dummy, 1, false);
        if (result>0) {
            printf("\nDevice found at 0x%02X. Bytes written: %d", addr,result);
        }
    }
}

// static void imu_mag_read(uint16_t magData[3]){
//     //Tell it to access magnetometer
//     //Read from magnetometer

//     //Slave 0 control
//     //The MPU-9250 communicates with its slave the magnetometer

//     //I2C_MST_CTRL 0x24
//     //Enable - MSB is 1
//     //LSBs are clock settings - I've just gone with 0
//     const uint8_t I2C_MST_CTRL_ADDR = 0x24;
//     const uint8_t I2C_MST_CTRL_ENABLE = 0x80;

//     //I2C_SLV0_ADDR 0x25
//     //Reading - MSB is 1
//     //From address 0x0C which is the magnetometer - LSB is 0x0C
//     const uint8_t I2C_SLV0_ADDR_VAL = 0x80|0x0C;

//     //I2C_SLV0_REG (0x26)
//     //Address of where to begin data transfer - Magnetometer values
//     const uint8_t I2C_SLV0_REG_VAL = 0x00; //Device ID register
//     //I2C_SLV0_EN (0x27)
//     //Enable reading data from the slave - MSB is 1 (0x80)
//     //The registers start at odd numbers set bit 4 to be 1
//     //Reading 6 bytes (2 bytes * x,y,z) - LSB is 6

//     uint8_t buffer[] = {0x24,0x80,0x0C|0x80,IMU_MAG_VALUES_ADDR,0x80|0b10000|0x6};
//     i2c_write_blocking(i2c,IMU_I2C_ADDR,buffer,5,false);

//     //It will start storing data at EXT_SENS_DATA_00
//     sleep_ms(10);
//     //Tell it that we want to access EXT_SENS_DATA_00
//     //We give it the register with a 1 at the MSB
//     //All registers are 7 bits long and a 1 MSB means read and a 0 MSB means write
//     uint8_t buffer2[] = {0x49|0x80};
//     i2c_write_blocking(i2c,IMU_I2C_ADDR,buffer2,1,false);

//     //The IMU then puts it in the FIFO and we read it
//     sleep_ms(10);
//     uint8_t magDataRaw[6] = {0};
//     i2c_read_blocking(i2c,IMU_I2C_ADDR,magDataRaw,6,false);

//     for (int i=0;i<3;i++) {
//         magData[i] = (magDataRaw[i*2] << 8 | magDataRaw[(i*2)+1]);
//     }
//     printf("\nMagnetometer - X: %d Y: %d Z: %d",magData[0],magData[1],magData[2]);

//     //Disable slave?
// }

static void imu_gyro_read(uint16_t gyroData[3]){
    const uint8_t GYRO_XOUT_H_ADDR = 0x43;
    uint8_t gyroDataRaw[6] = {0};
    mpu9250_read(GYRO_XOUT_H_ADDR,gyroDataRaw,6);
    for (int i=0;i<3;i++) {
        gyroData[i] = (gyroDataRaw[i*2] << 8 | gyroDataRaw[(i*2)+1]);
    }
    printf("\nGyroscope - X: %d Y: %d Z: %d",gyroData[0],gyroData[1],gyroData[2]);
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

    bool connected = mpu9250_test();
    if(!connected) return;
    

    imu_reset();

    imu_mag_test();
    imu_mag_bypass_scan();
    

    while(1){
        uint16_t gyroVals[3] = {0};
        imu_gyro_read(gyroVals);
        sleep_ms(1000);
    }

    
}