//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void getGres() {
    switch (Gscale) {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
    }
}

void getAres() {
    switch (Ascale) {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
    }
}


void readAccelData(int16_t * destination) {
    uint8_t rawData[6];  // x/y/z accel register data stored here
    readBytes(MPU9150_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


void readGyroData(int16_t * destination) {
    uint8_t rawData[6];  
    // x/y/z gyro register data stored here
    readBytes(MPU9150_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  
    // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  
    // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void readMagData(int16_t * destination) {
    uint8_t rawData[6];  
    // x/y/z gyro register data stored here
    writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x01); 
    // toggle enable data read from magnetometer, no continuous read mode!
    delay(10);
    // Only accept a new magnetometer data read if the data ready bit is set and 
    // if there are no sensor overflow or data read errors
    if(readByte(AK8975A_ADDRESS, AK8975A_ST1) & 0x01) {
        // wait for magnetometer data ready bit to be set
        readBytes(AK8975A_ADDRESS, AK8975A_XOUT_L, 6, &rawData[0]);  
        // Read the six raw data registers sequentially into data array
        destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  
        // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
        destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
    }
}

void initAK8975A(float * destination) {
    uint8_t rawData[3];  // x/y/z gyro register data stored here
    writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x00); // Power down
    delay(10);
    writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x0F); // Enter Fuse ROM access mode
    delay(10);
    readBytes(AK8975A_ADDRESS, AK8975A_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
    destination[0] =  (float)(rawData[0] - 128)/256. + 1.; // Return x-axis sensitivity adjustment values
    destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
    destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
}

int16_t readTempData() {
    uint8_t rawData[2];  // x/y/z gyro register data stored here
    readBytes(MPU9150_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
    return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

    // Configure the motion detection control for low power accelerometer mode
void LowPowerAccelOnlyMPU6050() {

    // The sensor has a high-pass filter necessary to invoke to allow the sensor motion detection algorithms work properly
    // Motion detection occurs on free-fall (acceleration below a threshold for some time for all axes), motion (acceleration
    // above a threshold for some time on at least one axis), and zero-motion toggle (acceleration on each axis less than a 
    // threshold for some time sets this flag, motion above the threshold turns it off). The high-pass filter takes gravity out
    // consideration for these threshold evaluations; otherwise, the flags would be set all the time!

    uint8_t c = readByte(MPU9150_ADDRESS, PWR_MGMT_1);
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, c & ~0x30); // Clear sleep and cycle bits [5:6]
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, c |  0x30); // Set sleep and cycle bits [5:6] to zero to make sure accelerometer is running

    c = readByte(MPU9150_ADDRESS, PWR_MGMT_2);
    writeByte(MPU9150_ADDRESS, PWR_MGMT_2, c & ~0x38); // Clear standby XA, YA, and ZA bits [3:5]
    writeByte(MPU9150_ADDRESS, PWR_MGMT_2, c |  0x00); // Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running

    c = readByte(MPU9150_ADDRESS, ACCEL_CONFIG);
    writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
    // Set high-pass filter to 0) reset (disable), 1) 5 Hz, 2) 2.5 Hz, 3) 1.25 Hz, 4) 0.63 Hz, or 7) Hold
    writeByte(MPU9150_ADDRESS, ACCEL_CONFIG,  c | 0x00);  // Set ACCEL_HPF to 0; reset mode disbaling high-pass filter

    c = readByte(MPU9150_ADDRESS, CONFIG);
    writeByte(MPU9150_ADDRESS, CONFIG, c & ~0x07); // Clear low-pass filter bits [2:0]
    writeByte(MPU9150_ADDRESS, CONFIG, c |  0x00);  // Set DLPD_CFG to 0; 260 Hz bandwidth, 1 kHz rate

    c = readByte(MPU9150_ADDRESS, INT_ENABLE);
    writeByte(MPU9150_ADDRESS, INT_ENABLE, c & ~0xFF);  // Clear all interrupts
    writeByte(MPU9150_ADDRESS, INT_ENABLE, 0x40);  // Enable motion threshold (bits 5) interrupt only

    // Motion detection interrupt requires the absolute value of any axis to lie above the detection threshold
    // for at least the counter duration
    writeByte(MPU9150_ADDRESS, MOT_THR, 0x80); // Set motion detection to 0.256 g; LSB = 2 mg
    writeByte(MPU9150_ADDRESS, MOT_DUR, 0x01); // Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate

    delay (100);  // Add delay for accumulation of samples

    c = readByte(MPU9150_ADDRESS, ACCEL_CONFIG);
    writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
    writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c |  0x07);  // Set ACCEL_HPF to 7; hold the initial accleration value as a referance

    c = readByte(MPU9150_ADDRESS, PWR_MGMT_2);
    writeByte(MPU9150_ADDRESS, PWR_MGMT_2, c & ~0xC7); // Clear standby XA, YA, and ZA bits [3:5] and LP_WAKE_CTRL bits [6:7]
    writeByte(MPU9150_ADDRESS, PWR_MGMT_2, c |  0x47); // Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])  

    c = readByte(MPU9150_ADDRESS, PWR_MGMT_1);
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, c & ~0x20); // Clear sleep and cycle bit 5
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, c |  0x20); // Set cycle bit 5 to begin low power accelerometer motion interrupts
}


void initMPU9150() {  
    // wake up device
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
    delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

    // get stable time source
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    delay(200);

    // Configure Gyro and Accelerometer
    // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
    // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
    // Minimum delay time is 4.9 ms which sets the fastest rate at ~200 Hz
    writeByte(MPU9150_ADDRESS, CONFIG, 0x03);  

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    writeByte(MPU9150_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c =  readByte(MPU9150_ADDRESS, GYRO_CONFIG);
    writeByte(MPU9150_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
    writeByte(MPU9150_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    writeByte(MPU9150_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro

    // Set accelerometer configuration
    c =  readByte(MPU9150_ADDRESS, ACCEL_CONFIG);
    writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
    writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer 

    /*
    // Configure Magnetometer for FIFO
    // Initialize AK8975A for write
    writeRegister(I2C_SLV1_ADDR, 0x0C);  // Write address of AK8975A
    writeRegister(I2C_SLV1_REG, 0x0A);   // Register from within the AK8975 to which to write
    writeRegister(I2C_SLV1_DO, 0x01);    // Register that holds output data written into Slave 1 when in write mode
    writeRegister(I2C_SLV1_CTRL, 0x81);  // Enable Slave 1

    // Set up auxilliary communication with AK8975A for FIFO read
    writeRegister(I2C_SLV0_ADDR, 0x8C); // Enable and read address (0x0C) of the AK8975A
    writeRegister(I2C_SLV0_REG, 0x03);  // Register within AK8975A from which to start data read
    writeRegister(I2C_SLV0_CTRL, 0x86); // Read six bytes and swap bytes

    // Configure FIFO
    writeRegister(INT_ENABLE, 0x00); // Disable all interrupts
    writeRegister(FIFO_EN, 0x00);    // Disable FIFO
    writeRegister(USER_CTRL, 0x02);  // Reset I2C master and FIFO and DMP
    writeRegister(USER_CTRL, 0x00);  // Disable FIFO 
    delay(100);
    // writeRegister(FIFO_EN, 0xF9); // Enable all sensors for FIFO 
    // writeRegister(I2C_MST_DELAY_CTRL, 0x80); // Enable delay of external sensor data until all data registers have been read
    */ 

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the Arduino as master
    writeByte(MPU9150_ADDRESS, INT_PIN_CFG, 0x22);    
    writeByte(MPU9150_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

    // Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
    // of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9150(float * dest1, float * dest2) {  
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device, reset all registers, clear gyro and accelerometer bias registers
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);  

    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x01);  
    writeByte(MPU9150_ADDRESS, PWR_MGMT_2, 0x00); 
    delay(200);

    // Configure device for bias calculation
    writeByte(MPU9150_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
    writeByte(MPU9150_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
    writeByte(MPU9150_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
    writeByte(MPU9150_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    writeByte(MPU9150_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    writeByte(MPU9150_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    writeByte(MPU9150_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    writeByte(MPU9150_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeByte(MPU9150_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
    writeByte(MPU9150_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
    delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    writeByte(MPU9150_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    readBytes(MPU9150_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        readBytes(MPU9150_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];          
    }

    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    if(accel_bias[2] > 0L) {
        accel_bias[2] -= (int32_t) accelsensitivity;
    }  // Remove gravity from the z-axis accelerometer bias calculation
    else {
        accel_bias[2] += (int32_t) accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    // Push gyro biases to hardware registers
    writeByte(MPU9150_ADDRESS, XG_OFFS_USRH, data[0]);
    writeByte(MPU9150_ADDRESS, XG_OFFS_USRL, data[1]);
    writeByte(MPU9150_ADDRESS, YG_OFFS_USRH, data[2]);
    writeByte(MPU9150_ADDRESS, YG_OFFS_USRL, data[3]);
    writeByte(MPU9150_ADDRESS, ZG_OFFS_USRH, data[4]);
    writeByte(MPU9150_ADDRESS, ZG_OFFS_USRL, data[5]);

    // Output scaled gyro biases for display in the main program
    dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
    dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    readBytes(MPU9150_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    readBytes(MPU9150_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    readBytes(MPU9150_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for(ii = 0; ii < 3; ii++) {
        if(accel_bias_reg[ii] & mask) 
            mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);
    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Push accelerometer biases to hardware registers
    writeByte(MPU9150_ADDRESS, XA_OFFSET_H, data[0]);
    writeByte(MPU9150_ADDRESS, XA_OFFSET_L_TC, data[1]);
    writeByte(MPU9150_ADDRESS, YA_OFFSET_H, data[2]);
    writeByte(MPU9150_ADDRESS, YA_OFFSET_L_TC, data[3]);
    writeByte(MPU9150_ADDRESS, ZA_OFFSET_H, data[4]);
    writeByte(MPU9150_ADDRESS, ZA_OFFSET_L_TC, data[5]);

    // Output scaled accelerometer biases for display in the main program
    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

    // Accelerometer and gyroscope self test; check calibration wrt factory settings
    // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
void MPU6050SelfTest(float * destination) {
    uint8_t rawData[4];
    uint8_t selfTest[6];
    float factoryTrim[6];

    // Configure the accelerometer for self-test
    writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
    writeByte(MPU9150_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    delay(250);  // Delay a while to let the device execute the self-test
    rawData[0] = readByte(MPU9150_ADDRESS, SELF_TEST_X); // X-axis self-test results
    rawData[1] = readByte(MPU9150_ADDRESS, SELF_TEST_Y); // Y-axis self-test results
    rawData[2] = readByte(MPU9150_ADDRESS, SELF_TEST_Z); // Z-axis self-test results
    rawData[3] = readByte(MPU9150_ADDRESS, SELF_TEST_A); // Mixed-axis self-test results
    // Extract the acceleration test results first
    selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
    selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
    selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer
    // Extract the gyration test results first
    selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
    selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
    selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer   
    // Process results to allow final comparison with factory set values
    factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
    factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
    factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
    factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation

    //  Output self-test results and factory trim calculation if desired
    //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
    //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
    //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
    //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 6; i++) {
        destination[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
    }

}

        // Wire.h read and write protocols
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress) {
    uint8_t data; // `data` will store the register data	 
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);	                 // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {  
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, count);  // Read bytes from slave register address 
    while (Wire.available()) {
        dest[i++] = Wire.read(); 
    }        
    // Put read results in the Rx buffer
}

/*
Subtract gravity from accelerometer readings in the body frame.
*/
void getRealAccel(VectorFloat accel) {
    realAccel.x = accel.x - (2.0f * (q[1] * q[3] - q[0] * q[2]));
    realAccel.y = accel.y - (2.0f * (q[0] * q[1] + q[2] * q[3]));
    realAccel.z = accel.z - (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
}

void getRealWorldAccel() {
	float tempQuat[4] = {0.0f, realAccel.x, realAccel.x, realAccel.x};
    tempQuat[0] = q[0]*tempQuat[0] - q[1]*tempQuat[1] - q[2]*tempQuat[2] - q[3]*tempQuat[3];
    tempQuat[1] = q[0]*tempQuat[1] + q[1]*tempQuat[0] + q[2]*tempQuat[3] - q[3]*tempQuat[2];
    tempQuat[2] = q[0]*tempQuat[2] - q[1]*tempQuat[3] + q[2]*tempQuat[0] + q[3]*tempQuat[1];
    tempQuat[3] = q[0]*tempQuat[3] + q[1]*tempQuat[2] - q[2]*tempQuat[1] + q[3]*tempQuat[0];
    float conjugate[4] = {q[0], -q[1], -q[2], -q[3]};
    tempQuat[0] = tempQuat[0]*conjugate[0] - tempQuat[1]*conjugate[1] - tempQuat[2]*conjugate[2] - tempQuat[3]*conjugate[3];
    tempQuat[1] = tempQuat[0]*conjugate[1] + tempQuat[1]*conjugate[0] + tempQuat[2]*conjugate[3] - tempQuat[3]*conjugate[2];
    tempQuat[2] = tempQuat[0]*conjugate[2] - tempQuat[1]*conjugate[3] + tempQuat[2]*conjugate[0] + tempQuat[3]*conjugate[1];
    tempQuat[3] = tempQuat[0]*conjugate[3] + tempQuat[1]*conjugate[2] - tempQuat[2]*conjugate[1] + tempQuat[3]*conjugate[0];
    worldAccel.x = tempQuat[1];
    worldAccel.y = tempQuat[2];
    worldAccel.z = tempQuat[3];
}

void getCorrectedMotion9(VectorFloat* accel, VectorFloat* gyro, VectorFloat* mag) {
    accelGyroMag.getMotion6(
        &accelCount.x, 
        &accelCount.y, 
        &accelCount.z,
        &gyroCount.x,
        &gyroCount.y,
        &gyroCount.z
    );

    //Now we'll calculate the accleration value into actual g's
    //get actual g value, this depends on scale being set
    accel->x = (float)accelCount.x*aRes;  
    accel->y = (float)accelCount.y*aRes;   
    accel->z = (float)accelCount.z*aRes;    

    //Calculate the gyro value into actual degrees per second
    //get actual gyro value, this depends on scale being set
    gyro->x = (float)gyroCount.x*gRes;  
    gyro->y = (float)gyroCount.y*gRes;  
    gyro->z = (float)gyroCount.z*gRes;  

    mcount++;
    if (mcount > 200/MagRate) {  
        //this is a poor man's way of setting the magnetometer read rate (see below) 
        readMagData(magCount);  // Read the x/y/z adc values
        
        //So far, magnetometer bias is calculated and subtracted here manually, should construct an algorithm to do it automatically
        //like the gyro and accelerometer biases
        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        mag->x = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
        mag->y = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];  
        mag->z = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];   
        mcount = 0;        
    }  
}

void magcalMPU9150(float * dest1) {
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0};
    int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0}, mag_temp[3] = {0, 0, 0};
    Serial.println("Mag Calibration: Wave device in a figure eight until done!");

    delay(4000);
    sample_count = 64;
    for(ii = 0; ii < sample_count; ii++) {
        readMagData(mag_temp); // Read the mag data
        for (int jj = 0; jj < 3; jj++) {
            if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
            if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }
        delay(135); // at 8 Hz ODR, new mag data is available every 125 ms
    }

    // Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
    // Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
    // Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);
    mag_bias[0] = (mag_max[0] + mag_min[0])/2; // get average x mag bias in counts
    mag_bias[1] = (mag_max[1] + mag_min[1])/2; // get average y mag bias in counts
    mag_bias[2] = (mag_max[2] + mag_min[2])/2; // get average z mag bias in counts
    dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0]; // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];
    dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];
    Serial.println("Mag Calibration done!");
}