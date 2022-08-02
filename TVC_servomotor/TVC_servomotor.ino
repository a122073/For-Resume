

#include "I2Cdev.h"
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

Servo SvRoll, SvPitch, SvYaw, Stest;


const int chipSelect = 4;

#define OUTPUT_READABLE_YAWPITCHROLL


bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];


Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];


int SdriveRoll, SdrivePitch, SdriveYaw;
int RollOffset = 0, PitchOffset = -2, YawOffset = 0;
int YawInitial = 0, YawSetInit = 0;




volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(9600);
  while (!Serial) {
    ;
  }


  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    //return;
    while (1);
  }
  Serial.println("card initialized.");

  Serial.println(F("Initializing Gyro sensor devices..."));
  mpu.initialize();


  Serial.println(F("Gyro device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();


  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);


  if (devStatus == 0) {

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {

    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }


  SvRoll.attach(9);
  SvPitch.attach(10);
  SvYaw.attach(5);


  SvRoll.write(90);
  SvPitch.write(90);
  SvYaw.write(90 + YawOffset);


  delay(1500);
}


void loop()
{

  if (!dmpReady) return;


  while (!mpuInterrupt && fifoCount < packetSize)
  {

  }


  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();


  fifoCount = mpu.getFIFOCount();


  if ((mpuIntStatus & 0x10) || fifoCount == 1024)

  {

    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

  }
  else if (mpuIntStatus & 0x02) {

    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();


    mpu.getFIFOBytes(fifoBuffer, packetSize);


    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    while (YawSetInit = 0)
    {
      YawInitial = int(ypr[0] * 180 / M_PI);
      YawSetInit = 1; // Already set.
    }


    SdriveRoll = int(ypr[2] * 180 / M_PI);
    SdrivePitch = int(ypr[1] * 180 / M_PI);
    SdriveYaw = (int(ypr[0] * 180 / M_PI)) - YawInitial;

    SvRoll.write(1.00 * (90 - SdriveRoll + RollOffset));
    SvPitch.write(1.00 * (90 - SdrivePitch + PitchOffset));

    Serial.println(SdriveRoll);
    Serial.print("\t");
    Serial.print(SdrivePitch);
    Serial.print("\t");
    Serial.print(SdriveYaw);
    Serial.print("\n");


    String dataString = "";


    for (int count = 9; count < 11; count++) {
      int sensor = analogRead(count);
      dataString += String(sensor);
      if (count < 10) {
        dataString += " , ";
      }
    }
    File dataFile = SD.open("datalog.txt", FILE_WRITE);

    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      Serial.println(dataString);
    }
    else {
      Serial.println("error opening datalog.txt");
    }

    /*timing = millis();
                if (start == 0)
                { timing1=timing;
                  start=1;
                }
                timing=timing-timing1;
                timing_sec = timing/1000;
                Serial.println(timing_sec);
                if((timing_sec)/5.0 == 0.0) // complete=1;
                if((timing_sec)/5.0 == 0.0 && (timing_sec)/10.0 == 0.0)  complete=0;*/

#endif
  }
}
