#include <Kalman.h>

/* Read quaternion and roll, pitch, yaw from MPU6050
* Robotics Course semester fall 2022 _ MiniProject #1
*/ 


#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
Kalman KFX;
Kalman KFY;
Kalman KFZ;

int16_t _accX, _accY, _accZ;
int16_t _gyroX, _gyroY, _gyroZ;

double roll_KF, pitch_KF, yaw_KF;  // Calculated angle using a kalman filter
double roll_CF, pitch_CF, yaw_CF;  // Calculated angle using a complementary filter

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format
// #define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO.
 #define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false;  
uint8_t devStatus;
uint8_t fifoBuffer[64];

Quaternion q;                 // [w, x, y, z]         quaternion container
VectorFloat gravity;          // [x, y, z]            gravity vector
float ypr[3];                 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint32_t timer;

// ================================================================
//                          INITIAL SETUP                       
// ================================================================

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        dmpReady = true;
    } 
    else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    timer = micros();    
}


// ================================================================
//                        MAIN PROGRAM LOOP                       
// ================================================================

void loop() {        
    if (!dmpReady) return;
    // read a packet from FIFO
    //
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.getMotion6(&_accX, &_accY, &_accZ, &_gyroX, &_gyroY, &_gyroZ);
            double accX = (double)_accX;
            double accY = (double)_accY;
            double accZ = (double)_accZ;
            double gyroX = (double)_gyroX;
            double gyroY = (double)_gyroY;
            double gyroZ = (double)_gyroZ;

            double dt = (double)(micros() - timer) / 1000000;  // Calculate delta time
            timer = micros();
            //Serial.println(dt);

            double gyroXrate = gyroX / 131.0;
            double gyroYrate = gyroY / 131.0;
            double gyroZrate = gyroZ / 131.0;

            float roll = ypr[2] * 180/M_PI;
            float pitch = ypr[1] * 180/M_PI;
            float yaw = ypr[0] * 180/M_PI;

            KFX.setAngle(roll);
            KFY.setAngle(pitch);
            KFZ.setAngle(yaw);

            roll_CF = roll;
            pitch_CF = pitch;
            yaw_CF = yaw;            
                        
            roll_KF = KFX.getAngle(roll, gyroXrate, dt);
            roll_CF = 0.93 * (roll_CF + gyroXrate * dt) + 0.07 * atan(accY / sqrt(accX * accX + accZ * accZ)) * 180/M_PI;;    

            pitch_KF = KFY.getAngle(pitch, gyroYrate, dt);
            pitch_CF = 0.93 * (pitch_CF + gyroYrate * dt) + 0.07 * atan2(-accX, accZ) * 180/M_PI;

            yaw_KF = KFZ.getAngle(yaw, gyroZrate, dt);
            yaw_CF = 0.93 * (yaw_CF + gyroZrate * dt) ;  
            
            //Serial.print(roll);
            //Serial.print(",");
            //Serial.print(roll_KF);
            //Serial.print(",");
            //Serial.println(roll_CF);                    

            //Serial.print(pitch);
            //Serial.print(",");
            //Serial.print(pitch_KF);
            //Serial.print(",");
            //Serial.println(pitch_CF);
            
            Serial.print(yaw);
            Serial.print(",");
            Serial.print(yaw_KF);
            Serial.print(",");
            Serial.println(yaw_CF);
        #endif
    }

}
