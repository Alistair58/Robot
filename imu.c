#include "imu.h"

static int icm20948_read(uint8_t reg,uint8_t *buffer,int len);
static int icm20948_write(uint8_t *buffer,int len);
static int icm20948_write_byte(uint8_t reg,uint8_t byte);
static bool icm20948_init();
static void private_mag_read(float mag_data[3],bool with_calibration);
static void private_gyro_read(float gyro_data[3],bool with_calibration);
static void private_accel_read(float accel_data[3],bool with_calibration);
static i2c_inst_t *i2c = i2c1;
static float gyro_calibration_offset[3] = {0};
static float mag_calibration_offset[3] = {0}; 
static float accel_calibration_offset[3] = {0};


static int icm20948_read(uint8_t reg,uint8_t *buffer,int len){
    int bytes_written = i2c_write_blocking(i2c,IMU_I2C_ADDR,&reg,1,false);
    if(bytes_written<=0){
        printf("\nCould not communicate with IMU. Error code: %d",bytes_written);
        return bytes_written;
    }
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
    return bytes_written;
}

static int icm20948_write_byte(uint8_t reg,uint8_t byte){
    uint8_t buffer[] = {reg,byte};
    int bytes_written = i2c_write_blocking(i2c, IMU_I2C_ADDR,buffer,2,false);
    if(bytes_written<=0){
        printf("\nCould not communicate with IMU. Error code: %d",bytes_written);
        return bytes_written;
    }
    return bytes_written;
}

static bool icm20948_init(){
    icm20948_write_byte(REG_BANK_SEL,BANK_0);
    uint8_t whoami_res = 0x00;
    icm20948_read(WHOAMI,&whoami_res,1);
    printf("\nWhoami: 0x%x",whoami_res);
    const uint8_t ICM20948_WHOAMI = 0xEA;
    if(whoami_res!=ICM20948_WHOAMI){ 
        printf("\nICM-20948 sent an invalid WHOAMI");
        return false;
    }
    //MSB to 1 resets the unit
    const uint8_t PWR_MGMT_RESET = 0x80;
    icm20948_write_byte(PWR_MGMT_1,PWR_MGMT_RESET);
    //Datasheet says to wait
    sleep_ms(100);

    //Reset puts it into sleep
    //We don't want sleep - set PWR_MGMT to default (no sleep)
    icm20948_write_byte(PWR_MGMT_1,0x01);

    //Set gyro sensitivity
    icm20948_write_byte(REG_BANK_SEL,BANK_2);
    //Enable low pass filter with -3DB BW at 23.9Hz and scale at +-500dps
    const uint8_t GYRO_CONFIG_VAL = 0b00100011;
    icm20948_write_byte(GYRO_CONFIG_1,GYRO_CONFIG_VAL);

    //Set accelerometer sensitivity
    //enable low pass filter with -3DB at 23.9Hz and scale at +-4g 
    const uint8_t ACCEL_CONFIG_VAL = 0b00100011;
    icm20948_write_byte(ACCEL_CONFIG,ACCEL_CONFIG_VAL);
    

    //Tell it to access magnetometer
    //The ICM-20948 communicates with its slave (0) the magnetometer
    icm20948_write_byte(REG_BANK_SEL,BANK_0);
    //Disable I2C bypass mode
    const uint8_t INT_PIN_CFG_VAL = 0x00;
    icm20948_write_byte(INT_PIN_CFG,INT_PIN_CFG_VAL); 
    //Enable the ICM_20948 being the master to the magnetometer
    const uint8_t USER_CTRL_MASTER_ENABLE = 0x20;
    icm20948_write_byte(USER_CTRL,USER_CTRL_MASTER_ENABLE);
    icm20948_write_byte(REG_BANK_SEL,BANK_3);
    icm20948_write_byte(I2C_MST_CTRL,0x07); //Set clock speed to 400kHz
    //Reading - MSB is 1
    //From address 0x0C which is the magnetometer - LSB is 0x0C
    icm20948_write_byte(I2C_SLV0_ADDR,0x80|AK09916_ADDR);
    icm20948_write_byte(I2C_SLV0_REG,DEVICE_ID);
    //Enable reading data from the slave - MSB is 1 (0x80)
    //Reading 1 byte - LSB is 1
    icm20948_write_byte(I2C_SLV0_CTRL,0x80|0x01);

    //Read the response from EXT_SLV_SENS_DATA
    icm20948_write_byte(REG_BANK_SEL,BANK_0);
    uint8_t mag_whoami_result = 0x00;
    icm20948_read(EXT_SLV_SENS_DATA_00,&mag_whoami_result,1);
    printf("\nAK09916 (Magnetometer chip) Whoami: %x",mag_whoami_result);
    const uint8_t AK09916_WHOAMI = 0x09;
    if(mag_whoami_result==AK09916_WHOAMI){
        printf("\nSuccessfully made contact with IMU");
        return true;
    }
    else{
        printf("\nAK09916 sent an invalid DEVICE_ID");
        return false;
    }
}

//Public-facing 
void mag_read(float mag_data[3]){
    private_mag_read(mag_data,true);
}

//Internal
static void private_mag_read(float mag_data[3],bool with_calibration){
    //Tell it to access magnetometer
    
    icm20948_write_byte(REG_BANK_SEL,BANK_3);
    //The ICM-20948 communicates with its slave (0) the magnetometer

    //Writing - MSB is 0
    //From address 0x0C which is the magnetometer - LSB is 0x0C
    icm20948_write_byte(I2C_SLV0_ADDR,AK09916_ADDR);
    icm20948_write_byte(I2C_SLV0_REG,CONTROL_2);
    icm20948_write_byte(I2C_SLV0_DO,0x01);  //single measurement mode
    //Enable - MSB is 1
    //Writing 1 byte - LSB is 1
    icm20948_write_byte(I2C_SLV0_CTRL,0x80|0x01);

    uint8_t data_ready = 0;
    uint32_t start_time = time_us_32();
    while(!(data_ready & 0x01)){ //DRDY bit
        if(time_us_32()-start_time>100000){
            //If we've waited 100ms, something's wrong
            printf("\nCannot obtain magnetometer readings");
            mag_data[0] = 0;
            mag_data[1] = 0;
            mag_data[2] = 0;
            return;
        }
        sleep_ms(10);
        //Read the status
        icm20948_write_byte(REG_BANK_SEL,BANK_3);
        //Reading - MSB is 1
        icm20948_write_byte(I2C_SLV0_ADDR,0x80|AK09916_ADDR);
        icm20948_write_byte(I2C_SLV0_REG,STATUS_1);
        //Enable reading data from the slave - MSB is 1 (0x80)
        //Reading 1 bytes - LSB is 1
        icm20948_write_byte(I2C_SLV0_CTRL,0x80|0x01);

        icm20948_write_byte(REG_BANK_SEL,BANK_0);
        icm20948_read(EXT_SLV_SENS_DATA_00,&data_ready,1);
    }   
    icm20948_write_byte(REG_BANK_SEL,BANK_3);
    //Reading - MSB is 1
    icm20948_write_byte(I2C_SLV0_ADDR,0x80|AK09916_ADDR);
    icm20948_write_byte(I2C_SLV0_REG,MEASUREMENT_DATA);
    //Enable reading data from the slave - MSB is 1 (0x80)
    //Reading 6 bytes - LSB is 6
    icm20948_write_byte(I2C_SLV0_CTRL,0x80|0x06);

    //It will start storing data at EXT_SENS_DATA_00
    icm20948_write_byte(REG_BANK_SEL,BANK_0);
    uint8_t mag_data_raw[6] = {0};
    icm20948_read(EXT_SLV_SENS_DATA_00,mag_data_raw,6);
    for (int i=0;i<3;i++){
        //little endian
        int16_t raw_value = ((int16_t) mag_data_raw[i*2+1] << 8 | mag_data_raw[(i*2)]);
        float micro_tesla = (float)raw_value*4912/(32752); //Max raw value is 32752 which is 4912 micro tesla
        mag_data[i] = micro_tesla;
        if(with_calibration){
            mag_data[i] -= mag_calibration_offset[i];
        }
    }
    printf("\nMagnetometer - X: %f Y: %f Z: %f",mag_data[0],mag_data[1],mag_data[2]);
}

void calibrate_mag_blocking(void){
    const int num_samples = 50;
    //Read 50 mag vals over around 1s
    float mag_read_vals[3] = {0};
    for(int i=0;i<3;i++){
        mag_calibration_offset[i] = 0;
    }
    for(int i=0;i<num_samples;i++){
        //The function has a 10ms sleep in it
        private_mag_read(mag_read_vals,false);
        for(int i=0;i<3;i++){
            mag_calibration_offset[i] += mag_read_vals[i];
        }
    }
    for(int i=0;i<3;i++){
        mag_calibration_offset[i] /= num_samples;
    }
}

//Public-facing
void gyro_read(float gyro_data[3]){
    private_gyro_read(gyro_data,true);
}

//Internal
static void private_gyro_read(float gyro_data[3],bool with_calibration){
    icm20948_write_byte(REG_BANK_SEL,BANK_0);
    uint8_t gyro_data_raw[6] = {0};
    icm20948_read(GYRO_XOUT_H,gyro_data_raw,6);
    for (int i=0;i<3;i++) {
        //big endian
        int16_t raw_value = ((int16_t)gyro_data_raw[i*2] << 8 | gyro_data_raw[(i*2)+1]);
        float dps = (float)raw_value*500/(1<<15);
        gyro_data[i] = dps;
        if(with_calibration){
            gyro_data[i] -= gyro_calibration_offset[i];
        }
    }
    printf("\nGyroscope - X: %f Y: %f Z: %f",gyro_data[0],gyro_data[1],gyro_data[2]);
}

void calibrate_gyro_blocking(void){
    const int num_samples = 50;
    //Read 50 gyro vals over around 1s
    float gyro_read_vals[3] = {0};
    for(int i=0;i<3;i++){
        gyro_calibration_offset[i] = 0;
    }
    for(int i=0;i<num_samples;i++){
        sleep_ms(10);
        private_gyro_read(gyro_read_vals,false);
        for(int i=0;i<3;i++){
            gyro_calibration_offset[i] += gyro_read_vals[i];
        }
    }
    for(int i=0;i<3;i++){
        gyro_calibration_offset[i] /= num_samples;
    }
}

//Public-facing
void accel_read(float accel_data[3]){
    private_accel_read(accel_data,true);
}

//Internal
static void private_accel_read(float accel_data[3],bool with_calibration){
    icm20948_write_byte(REG_BANK_SEL,BANK_0);
    uint8_t accel_data_raw[6] = {0};
    icm20948_read(ACCEL_XOUT_H,accel_data_raw,6);
    for (int i=0;i<3;i++) {
        //big endian
        int16_t raw_value = ((int16_t)accel_data_raw[i*2] << 8 | accel_data_raw[(i*2)+1]);
        //Max value is 4g
        float ms2 = (float)raw_value*4*9.81/(1<<15); //ms^-2
        accel_data[i] = ms2;
        if(with_calibration){
            accel_data[i] -= accel_calibration_offset[i];
        }
    }
    printf("\nAccelerometer - X: %f Y: %f Z: %f",accel_data[0],accel_data[1],accel_data[2]);
}

void calibrate_accel_blocking(void){
    const int num_samples = 50;
    //Read 50 gyro vals over around 1s
    float accel_read_vals[3] = {0};
    for(int i=0;i<3;i++){
        accel_calibration_offset[i] = 0;
    }
    for(int i=0;i<num_samples;i++){
        sleep_ms(10);
        private_accel_read(accel_read_vals,false);
        for(int i=0;i<3;i++){
            accel_calibration_offset[i] += accel_read_vals[i];
        }
    }
    for(int i=0;i<3;i++){
        accel_calibration_offset[i] /= num_samples;
    }
}

void imu_i2c_init(void){
    i2c_init(i2c,400*1000); //400kHz
    gpio_set_function(IMU_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_I2C_SDA_PIN);
    gpio_pull_up(IMU_I2C_SCL_PIN);
    bool connected = icm20948_init();
    if(!connected) return;
    calibrate_gyro_blocking();
    calibrate_mag_blocking();
    calibrate_accel_blocking();
}