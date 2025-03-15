#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

bool dmpReady = false;  
uint8_t devStatus;
uint8_t fifoBuffer[64];

Quaternion q;                 // [w, x, y, z]         quaternion container
VectorFloat gravity;          // [x, y, z]            gravity vector
float ypr[3];                 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int16_t _accX, _accY, _accZ;
int16_t _gyroX, _gyroY, _gyroZ;

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);

    mpu.initialize();

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);

        dmpReady = true;  
              
    }    
  }

void loop() {
   if (!dmpReady) return;
   if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
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

        Serial.print(accX);
        Serial.print(",");
        Serial.print(accY);
        Serial.print(",");
        Serial.print(accZ);
        Serial.print(",");
        Serial.print(gyroX);
        Serial.print(",");
        Serial.print(gyroY);
        Serial.print(",");
        Serial.println(gyroZ);
   }

}
