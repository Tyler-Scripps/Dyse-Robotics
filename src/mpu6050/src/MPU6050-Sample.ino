#include "MPU6050.h"

MPU6050 mpu;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Initialized Program!");
  mpu.begin();
}
void loop() {

  Vector rawA, rawG;
  mpu.readRawAccel(&rawA);
  mpu.readRawGyro(&rawG);
  mpu.test_WhoAmI();
  
  Serial.print("WhoAmI test; "); Serial.println(mpu.getState());
  Serial.print("MPU6050 Accel Raw: (");
  Serial.print(rawA.X); Serial.print(", "); Serial.print(rawA.Y); Serial.print(", "); Serial.print(rawA.Z); Serial.println(")");
  Serial.print("MPU6050 Gyro Raw: (");
  Serial.print(rawG.X); Serial.print(", "); Serial.print(rawG.Y); Serial.print(", "); Serial.print(rawG.Z); Serial.println(")");
}
