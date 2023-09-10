#include <Arduino.h>
#include <Wire.h>
#include <ICM40627.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
 
ICM40627 IMU;
SFE_MMC5983MA MAG;

int16_t IMU_data;
uint32_t currentX = 0;
uint32_t currentY = 0;
uint32_t currentZ = 0;

void setup() {
  Wire.begin(4,5);
  IMU.init(AODR_200Hz, GODR_200Hz, AFS_8G, GFS_2000DPS);
  MAG.begin();
  MAG.softReset();
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
}
 
void loop() {
  IMU.readData(&IMU_data);
  Serial.println("IMU data:");
  Serial.println(IMU_data);

  currentX = MAG.getMeasurementX();
  currentY = MAG.getMeasurementY();
  currentZ = MAG.getMeasurementZ();
  Serial.println("MAG data:");
  Serial.println(currentX);
  Serial.println(currentY);
  Serial.println(currentZ);
  Serial.println();

  delay(1000);          
}