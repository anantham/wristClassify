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

// Using the GY-521 breakout board, I set ADO to 0 by grounding through a 4k7 resistor
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 0
#if ADO
    #define MPU9150_ADDRESS 0x69  // Device address when ADO = 1
#else
    #define MPU9150_ADDRESS 0x68  // Device address when ADO = 0
#define AK8975A_ADDRESS 0x0C //  Address of magnetometer
#endif  

#define AHRS  false          
#define SerialDebug true 
#define wifi false
#include "MPU9150.h"
#include "helper_3dmath.h"
#include "Wire.h" 
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

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
float aRes, gRes, mRes = 10.*1229./4096.; // scale resolutions per LSB for the sensors

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
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
VectorFloat accel, gyro, mag, realAccel, worldAccel, oldWorldAccel;
ESP8266WiFiMulti WiFiMulti;
WiFiClient client;
MPU9150 accelGyroMag;

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(4, 5);
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    WiFiMulti.addAP(SSID, Password);
    Serial.begin(115200);

    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(intPin, INPUT);
    digitalWrite(intPin, LOW);

    if (accelGyroMag.testConnection()) {  
        Serial.println("MPU9150 is online...");
        getAres();
        getGres();

        //Calibrate gyro and accelerometers, load biases in bias registers 
        //calibrateMPU9150(gyroBias, accelBias);

        //Calibrated values for my MPU9150, run calibrateMPU9150 for other sensors.

        
        accelGyroMag.setXAccelOffset(-3304);
        accelGyroMag.setYAccelOffset(1788);
        accelGyroMag.setZAccelOffset(1878);
        accelGyroMag.setXGyroOffset(124);
        accelGyroMag.setYGyroOffset(104);
        accelGyroMag.setZGyroOffset(-119);
        

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
        MagRate = 50; 
        magbias[0] = 181.92;   // User environmental x-axis correction in milliGauss
        magbias[1] = -297.28;  // User environmental y-axis correction in milliGauss
        magbias[2] = 25.14;
        if(wifi) {
            Serial.print("Wait for WiFi... ");
            while(WiFiMulti.run() != WL_CONNECTED) {
                Serial.print(".");
                delay(500);
            }
            Serial.println("");
            Serial.println("WiFi connected");
            Serial.println("IP address: ");
            Serial.println(WiFi.localIP());
            Serial.print("connecting to ");
            Serial.println(host);
            if (!client.connect(host, port)) {
                Serial.println("connection failed");
                Serial.println("wait 5 sec...");
                delay(5000);
                return;
            }
            client.setNoDelay(true);
        }
    }
    else {
        Serial.println("MPU9150 connection failed.");
        while(1);
    }
}

void loop() {  
    // If intPin goes high or data ready status is TRUE, all data registers have new data
    if (accelGyroMag.getIntDataReadyStatus()) {  
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
    MahonyQuaternionUpdate(
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
    oldWorldAccel.x = worldAccel.x;
    oldWorldAccel.y = worldAccel.y;
    oldWorldAccel.z = worldAccel.z;
    getRealAccel(accel);
    getRealWorldAccel();
    complimentaryFilterUpdate(&worldAccel, &oldWorldAccel, 0.9);
    if (!AHRS) {
        int delt_t = millis() - count; 
        String output = String(1000*worldAccel.x, 2) + 
        ',' + String(1000*worldAccel.y, 2) + 
        ',' + String(1000*worldAccel.z, 2);
        if(SerialDebug) {
            Serial.println(output);
        }   
        if(wifi) {
            while(!client.connected()) {
                client.connect(host, port);
                client.setNoDelay(true);
            }
            client.println(output);
        }               
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
    }
    count=millis();
}