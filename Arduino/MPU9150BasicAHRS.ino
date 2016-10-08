/* 
MPU9150 Basic Example Code
by: Kris Winer
date: March 1, 2014
license: Beerware - Use this code however you'd like. If you 
find it useful you can buy me a beer some time.
Demonstrate MPU-9150 basic functionality including parameterizing the register addresses, initializing the sensor, 
etting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

SDA and SCL should have external pull-up resistors (to 3.3V).
10k resistors are on the GY-9150 breakout board.

Hardware setup:
MPU9150 Breakout --------- Arduino
3.3V --------------------- 3.3V
SDA ----------------------- A4
SCL ----------------------- A5
GND ---------------------- GND

Note: The MPU9150 is an I2C sensor and uses the Arduino Wire library. 
Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
*/
// Define registers per MPU6050, Register Map and Descriptions, Rev 4.2, 08/19/2013 6 DOF Motion sensor fusion device
// Invensense Inc., www.invensense.com
// See also MPU-9150 Register Map and Descriptions, Revision 4.0, RM-MPU-9150A-00, 9/12/2012 for registers not listed in 
// above document; the MPU6050 and MPU 9150 are virtually identical but the latter has an on-board magnetic sensor
//
//Magnetometer Registers

#define WHO_AM_I_AK8975A 0x00 // should return 0x48
#define INFO             0x01
#define AK8975A_ST1      0x02  // data ready status bit 0
#define AK8975A_ADDRESS  0x0C
#define AK8975A_XOUT_L   0x03  // data
#define AK8975A_XOUT_H   0x04
#define AK8975A_YOUT_L   0x05
#define AK8975A_YOUT_H   0x06
#define AK8975A_ZOUT_L   0x07
#define AK8975A_ZOUT_H   0x08
#define AK8975A_ST2      0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8975A_CNTL     0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8975A_ASTC     0x0C  // Self test control
#define AK8975A_ASAX     0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8975A_ASAY     0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8975A_ASAZ     0x12  // Fuse ROM z-axis sensitivity adjustment value

#define XGOFFS_TC        0x00 // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD                 
#define YGOFFS_TC        0x01                                                                          
#define ZGOFFS_TC        0x02
#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E    
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
#define XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope, populate with calibration routine
#define XG_OFFS_USRL     0x14
#define YG_OFFS_USRH     0x15
#define YG_OFFS_USRL     0x16
#define ZG_OFFS_USRH     0x17
#define ZG_OFFS_USRL     0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FF_THR           0x1D  // Free-fall
#define FF_DUR           0x1E  // Free-fall
#define MOT_THR          0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL   0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9150 0x75 // Should return 0x68

// Using the GY-521 breakout board, I set ADO to 0 by grounding through a 4k7 resistor
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 0
#if ADO
    #define MPU9150_ADDRESS 0x69  // Device address when ADO = 1
#else
    #define MPU9150_ADDRESS 0x68  // Device address when ADO = 0
#define AK8975A_ADDRESS 0x0C //  Address of magnetometer
#endif  

#include "MPU9150.h"
#include "helper_3dmath.h"
#include "Wire.h" 

#define AHRS  false          
#define SerialDebug true  

// Set initial input parameters
enum Ascale {
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

enum Gscale {
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
float aRes, gRes, mRes; // scale resolutions per LSB for the sensors

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins

//int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
//int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;     // Stores the raw internal chip temperature counts
float temperature;     // temperature in degrees Centigrade
float SelfTest[6];

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
// gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasError = PI * (40.0f / 180.0f);  
// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   
/*
There is a tradeoff in the beta parameter between accuracy and response speed.
In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 
2.7 degrees/s) was found to give optimal accuracy.However, with this value, the LSM9SD0
response time is about 10 seconds to a stable initial quaternion. Subsequent changes also 
require a longish lag time to a stable output, not fast enough for a quadcopter or robot 
car! By increasing beta (GyroMeasError) by about a factor of fifteen, the response time 
constant is reduced to ~2 sec I haven't noticed any reduction in solution accuracy. This 
is essentially the I coefficient in a PID control sense; the bigger the feedback 
coefficient, the faster the solution converges, usually at the expense of accuracy. 
In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
*/

float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0;  // used to control display output rate
uint32_t mcount = 0; // used to control magnetometer read rate
uint32_t MagRate;    // read rate for magnetometer data

float pitch, yaw, roll;
float deltat = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
float a31, a32, a33;

VectorInt16 gyroCount; 
VectorInt16 accelCount;
VectorFloat accel, gyro, mag, realAccel, worldAccel;

MPU9150 accelGyroMag;

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(4, 5);
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(intPin, INPUT);
    digitalWrite(intPin, LOW);

    if (accelGyroMag.testConnection()) {  
        Serial.println("MPU9150 is online...");
        getAres();
        getGres();
        // Conversion from 1229 microTesla full scale (4096) to 12.29 Gauss full scale
        mRes = 10.*1229./4096.; 
        MPU6050SelfTest(SelfTest); 
        if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
            delay(1000);
        }

        //Calibrate gyro and accelerometers, load biases in bias registers 
        calibrateMPU9150(gyroBias, accelBias);

        //Calibrated values for my MPU9150, run calibrateMPU9150 for other sensors.

        /*
        accelGyroMag.setXAccelOffset(-1340);
        accelGyroMag.setYAccelOffset(1802);
        accelGyroMag.setZAccelOffset(820);
        accelGyroMag.setXGyroOffset(-33);
        accelGyroMag.setYGyroOffset(-28);
        accelGyroMag.setZGyroOffset(32);
        */

        // Inititalize and configure accelerometer and gyroscope
        accelGyroMag.initialize();
        // Initialize device for active mode read of acclerometer, gyroscope, and temperature
        Serial.println("MPU9150 initialized for active data mode...."); 

        // Read the WHO_AM_I register of the magnetometer, this is a good test of communication  
        uint8_t c = readByte(AK8975A_ADDRESS, WHO_AM_I_AK8975A);  
        delay(1000); 

        // Get magnetometer calibration from AK8975A ROM
        initAK8975A(magCalibration); 
        //magcalMPU9150(magbias);
        // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values
        MagRate = 75; 
        magbias[0] = 181.92;   // User environmental x-axis correction in milliGauss
        magbias[1] = -297.28;  // User environmental y-axis correction in milliGauss
        magbias[2] = 25.14;
    }
    else {
        Serial.println("MPU9150 connection failed.");
        while(1);
    }
}

void loop() {  
    // If intPin goes high or data ready status is TRUE, all data registers have new data
    if (readByte(MPU9150_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
        getCorrectedMotion9(&accel, &gyro, &mag);
    }

    //set integration time by time elapsed since last filter update
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); 
    lastUpdate = Now;

    /*
    Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
    the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
    We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
    For the MPU-9150, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
    in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
    This is ok by aircraft orientation standards!  
    Pass gyro rate as rad/s
    */

    //MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
    MadgwickQuaternionUpdate(
        accel.x, 
        accel.y, 
        accel.z, 
        gyro.x*PI/180.0f, 
        gyro.y*PI/180.0f, 
        gyro.z*PI/180.0f,  
        mag.y,  
        mag.x, 
        mag.z
    );

    getRealAccel(accel);
    getRealWorldAccel();
    //complementaryFilterUpdate(realWorldAccel, 0.995);
      
    if (!AHRS) {
        int delt_t = millis() - count; 
        if(SerialDebug) {
            String output = String(1000*worldAccel.x, 2) + ',' + String(1000*worldAccel.y, 2) + ',' + String(1000*worldAccel.z, 2);
            Serial.println(output);
        }      
        count = millis(); 
    }
    else {        
        delt_t = millis() - count;

        /*
        Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
        In this coordinate system, the positive z-axis is down toward Earth. 
        Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
        Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
        Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
        These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
        Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
        applied in the correct order which for this configuration is yaw, pitch, and then roll.
        For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
        */

        yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
        pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI; 
        roll  *= 180.0f / PI;

        if(SerialDebug) {
            //Serial.print("Yaw, Pitch, Roll: ");
            Serial.print(yaw, 2);
            Serial.print(", ");
            Serial.print(pitch, 2);
            Serial.print(", ");
            Serial.println(roll, 2);
            //Serial.print("average rate = "); Serial.print(1.0f/deltat, 2); Serial.println(" Hz");
        }

        /*
        With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and 
        >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
        The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
        the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
        an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
        filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. 
        This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
        This filter update rate should be fast enough to maintain accurate platform orientation for 
        stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
        produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
        The 3.3 V 8 MHz Pro Mini is doing pretty well!
        Display 0.5-second average filter rate
        */

        count = millis();  
    }
}