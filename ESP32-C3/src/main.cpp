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

bool debug = true;

WiFiClient wifiClient;
MqttClient mqtt(wifiClient);

ICM40627 IMU;
SFE_MMC5983MA MAG;
Madgwick MadgwickFilter;
Mahony MahonyFilter;

char topic_dataMadgwick[] = "AHRS/data_Madgwick";
char topic_rawdata[] = "rawdata";
char topic_dataMahony[] = "AHRS/data_dataMahony";

void sendMqttMsg(String to_print, String topic, MqttClient& mqtt, bool debug){
    mqtt.beginMessage(topic);
    mqtt.print(to_print);
    mqtt.endMessage();
    if(debug){
        Serial.println(String(to_print + " : to : " + String(topic)));
    }
}

struct MeasurementData {
    float accelerometer_x;
    float accelerometer_y;
    float accelerometer_z;
    float gyroscope_x;
    float gyroscope_y;
    float gyroscope_z;
    float mag_x;
    float mag_y;
    float mag_z;
};

struct AHRSData {
    float roll;
    float pitch;
    float yaw;
};

AHRSData MadgwickResult;
AHRSData MahonyResult;
AHRSData KalmanResult;
AHRSData ComplementarFilterResult;

unsigned long LoopTime;
LowPassFilter *filters[6];
float b_coeff[3] = {0.04763686, 0.09527372, 0.04763686};
float a_coeff[2] = {1.29479367, -0.48534111};

void filterData(MeasurementData &incommingData, MeasurementData &filteredData){
    filters[0]->calculate_output(incommingData.accelerometer_x, &filteredData.accelerometer_x);
    filters[1]->calculate_output(incommingData.accelerometer_y, &filteredData.accelerometer_y);
    filters[2]->calculate_output(incommingData.accelerometer_z, &filteredData.accelerometer_z);
    filters[3]->calculate_output(incommingData.gyroscope_x, &filteredData.gyroscope_x);
    filters[4]->calculate_output(incommingData.gyroscope_y, &filteredData.gyroscope_y);
    filters[5]->calculate_output(incommingData.gyroscope_z, &filteredData.gyroscope_z);
}


// Define calibration parameters
float A[3][3] = {
    {1.157036, -0.003114, -0.001709},
    {-0.003114, 1.254286, 0.058145},
    {-0.001709, 0.058145, 1.275776}
};
float b[3] = {2.311279, 0.047097, -20.687891};


MeasurementData update_measurements(){
  MeasurementData data;
  int16_t IMU_data[7];
  IMU.readData(IMU_data);
  data.accelerometer_x = IMU_data[1] * IMU.getAres(AFS_8G);
  data.accelerometer_y = IMU_data[2] * IMU.getAres(AFS_8G);
  data.accelerometer_z = IMU_data[3] * IMU.getAres(AFS_8G);

  data.gyroscope_x = IMU_data[4] * IMU.getAres(GFS_2000DPS);
  data.gyroscope_y = IMU_data[5] * IMU.getAres(GFS_2000DPS);
  data.gyroscope_z = IMU_data[6] * IMU.getAres(GFS_2000DPS);

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

  data.mag_x = -data.mag_x;
  // data.mag_x = data.mag_x;
  data.mag_y = data.mag_y;
  // data.mag_z = data.mag_z;
  data.mag_z = -data.mag_z;

  data.mag_x *= 100;
  data.mag_y *= 100;
  data.mag_z *= 100;

  float calibData[3];
  float currMeas[3];
  currMeas[0] = data.mag_x;
  currMeas[1] = data.mag_y;
  currMeas[2] = data.mag_z;
  for (int j = 0; j < 3; j++) {
    calibData[j] = 0;
    for (int k = 0; k < 3; k++) {
      calibData[j] += A[j][k] * (currMeas[k] - b[k]);
    }
  }
  data.mag_x = calibData[0];
  data.mag_y = calibData[1];
  data.mag_z = calibData[2];


  MeasurementData temp;
  filterData(data, temp);
  data.gyroscope_x = temp.gyroscope_x;
  data.gyroscope_y = temp.gyroscope_y;
  data.gyroscope_z = temp.gyroscope_z;
  data.accelerometer_x = temp.accelerometer_x;
  data.accelerometer_y = temp.accelerometer_y;
  data.accelerometer_z = temp.accelerometer_z;

  // Serial.print(data.accelerometer_x);
  // Serial.print(", ");
  // Serial.print(data.accelerometer_y);
  // Serial.print(", ");
  // Serial.print(data.accelerometer_z);
  // Serial.print("\t");

  // Serial.print(data.gyroscope_x);
  // Serial.print(", ");
  // Serial.print(data.gyroscope_y);
  // Serial.print(", ");
  // Serial.print(data.gyroscope_z);
  // Serial.print("\t");

  // Serial.print(data.mag_x);
  // Serial.print(", ");
  // Serial.print(data.mag_y);
  // Serial.print(", ");
  // Serial.print(data.mag_z);
  // Serial.println("");

  return data;
}

void IMU_calib(){
  int16_t IMU_data[7];
  float gyroscope_x;
  float gyroscope_y;
  float gyroscope_z;
  float sumx = 0;
  float sumy = 0;
  float sumz = 0;
  float offsetX = 0;
  float offsetY = 0;
  float offsetZ = 0;
  for(int i=0; i < 50; i++){
    IMU.readData(IMU_data);
    gyroscope_x = IMU_data[4] * IMU.getAres(GFS_2000DPS);
    gyroscope_y = IMU_data[5] * IMU.getAres(GFS_2000DPS);
    gyroscope_z = IMU_data[6] * IMU.getAres(GFS_2000DPS);
    sumx += gyroscope_x;
    sumy += gyroscope_y;
    sumz += gyroscope_z;
    delay(5);
  }
  offsetX = sumx/50;
  offsetY = sumy/50;
  offsetZ = sumz/50;
  IMU.setOffsets(0.,0.,0.,offsetX,offsetY,offsetZ);
}

void setup() {
  Wire.begin(4,5);
  Serial.begin(115200);


  IMU.init(AFS_8G, GFS_2000DPS, AODR_100Hz, GODR_100Hz);
  IMU_calib();
  for (int filter_id =0; filter_id< 6;filter_id++){
    filters[filter_id] = new LowPassFilter(a_coeff,b_coeff);
  }

  MAG.begin();
  MAG.softReset();
  MAG.setFilterBandwidth(100);
  MAG.setContinuousModeFrequency(100);
  MAG.enableContinuousMode();


  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PSWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("Connecting to MQTT host");
  mqtt.setUsernamePassword(
    MQTT_USER,
    MQTT_PSWD
  );
  if (!mqtt.connect(MQTT_BROKER_IP, MQTT_BROKER_PORT)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqtt.connectError());
    while (1);
  }
  Serial.println("MQTT connected");


  MadgwickFilter.begin(100);
  MahonyFilter.begin(100);


  LoopTime = 1000000 / 100;
}
 
unsigned long currentTime;
unsigned long dt;
unsigned long prevTime = 0;
MeasurementData data;

void loop() {
  currentTime = micros();
	dt = currentTime - prevTime; 
  if (dt >= LoopTime) {
    // Serial.println(dt);
    prevTime = currentTime;

    data = update_measurements();
    // MadgwickFilter.update(data.gyroscope_x, data.gyroscope_y, data.gyroscope_z, data.accelerometer_x, data.accelerometer_y, data.accelerometer_z, data.mag_x, data.mag_y, data.mag_z);
    // MadgwickFilter.updateIMU(data.gyroscope_x, data.gyroscope_y, data.gyroscope_z, data.accelerometer_x, data.accelerometer_y, data.accelerometer_z);

    // MadgwickResult.roll = MadgwickFilter.getRoll();
    // MadgwickResult.pitch = MadgwickFilter.getPitch();
    // MadgwickResult.yaw = MadgwickFilter.getYaw();

    // MahonyFilter.update(data.gyroscope_x, data.gyroscope_y, data.gyroscope_z, data.accelerometer_x, data.accelerometer_y, data.accelerometer_z, data.mag_x, data.mag_y, data.mag_z);

    // MahonyResult.roll = MahonyFilter.getRoll();
    // MahonyResult.pitch = MahonyFilter.getPitch();
    // MahonyResult.yaw = MahonyFilter.getYaw();

    String data_to_print;
    data_to_print = String(MadgwickResult.roll)+';'+String(MadgwickResult.pitch)+';'+String(MadgwickResult.yaw);
    data_to_print = String(data.accelerometer_x)+';'+String(data.accelerometer_y)+';'+String(data.accelerometer_z)+';'+String(data.gyroscope_x)+';'+String(data.gyroscope_y)+';'+String(data.gyroscope_z)+';'+String(data.mag_x)+';'+String(data.mag_y)+';'+String(data.mag_z);
    sendMqttMsg(data_to_print, topic_rawdata, mqtt, debug);

    // Serial.print(data.mag_x, 6); Serial.print(",");
    // Serial.print(data.mag_y, 6); Serial.print(",");
    // Serial.println(data.mag_z, 6);

    // data_to_print = String(MahonyResult.roll)+';'+String(MahonyResult.pitch)+';'+String(MahonyResult.yaw);
    // sendMqttMsg(data_to_print, topic_dataMahony, mqtt, debug);
  }
}