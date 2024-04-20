#include <Arduino.h>
#include <Wire.h>
#include <ICM40627.h>
#include <MadgwickAHRS.h>
#include <MahonyAHRS.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <ArduinoMqttClient.h>
#include "credentials.h"
#include <WiFi.h>
#include "LowPassFilter.h"

bool debug = false;

SFE_MMC5983MA MAG;

struct MeasurementData {
    float mag_x;
    float mag_y;
    float mag_z;
};


unsigned long LoopTime;

MeasurementData update_measurements(){
  MeasurementData data;

  uint32_t MAGX;
  uint32_t MAGY;
  uint32_t MAGZ;

  MAG.readFieldsXYZ(&MAGX, &MAGY, &MAGZ);

  data.mag_x = (double)MAGX - 131072.0;
  data.mag_x /= 131072.0;
  data.mag_y = (double)MAGY - 131072.0;
  data.mag_y /= 131072.0;
  data.mag_z = (double)MAGZ - 131072.0;
  data.mag_z /= 131072.0;

  data.mag_x = data.mag_x * 8;
  data.mag_y = data.mag_y * 8;
  data.mag_z = data.mag_z * 8;

  // data.mag_x = -data.mag_x;
  data.mag_x = data.mag_x;
  data.mag_y = data.mag_y;
  data.mag_z = data.mag_z;
  // data.mag_z = -data.mag_z;

  return data;
}

MeasurementData data;
float min_x, max_x, mid_x;
float min_y, max_y, mid_y;
float min_z, max_z, mid_z;

void setup() {
  Wire.begin(4,5);
  Serial.begin(115200);

  MAG.begin();
  MAG.softReset();
  MAG.setFilterBandwidth(100);
  MAG.setContinuousModeFrequency(100);
  MAG.enableContinuousMode();

  data = update_measurements()
  min_x = max_x = data.mag_x;
  min_y = max_y = data.mag_y;
  min_z = max_z = data.mag_z;
  delat(10);
}
 
unsigned long currentTime;
unsigned long dt;
unsigned long prevTime = 0;

void loop() {
  data = update_measurements()
  float x = data.mag_x;
  float y = data.mag_y;
  float z = data.mag_z;
  
  Serial.print("Mag: (");
  Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.print(", ");
  Serial.print(z); Serial.print(")");

  min_x = min(min_x, x);
  min_y = min(min_y, y);
  min_z = min(min_z, z);

  max_x = max(max_x, x);
  max_y = max(max_y, y);
  max_z = max(max_z, z);

  mid_x = (max_x + min_x) / 2;
  mid_y = (max_y + min_y) / 2;
  mid_z = (max_z + min_z) / 2;
  Serial.print(" Hard offset: (");
  Serial.print(mid_x); Serial.print(", ");
  Serial.print(mid_y); Serial.print(", ");
  Serial.print(mid_z); Serial.print(")");  

  Serial.print(" Field: (");
  Serial.print((max_x - min_x)/2); Serial.print(", ");
  Serial.print((max_y - min_y)/2); Serial.print(", ");
  Serial.print((max_z - min_z)/2); Serial.println(")");    
  delay(10); 
}