#include <Arduino.h>
#include <Wire.h>
#include <ICM40627.h>
#include <MadgwickAHRS.h>
#include <MahonyAHRS.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <ArduinoMqttClient.h>
#include "credentials.h"
#include <WiFi.h>

bool debug = true;

WiFiClient wifiClient;
MqttClient mqtt(wifiClient);

ICM40627 IMU;
SFE_MMC5983MA MAG;
Madgwick MadgwickFilter;
Mahony MahonyFilter;

char topic_dataMadgwick[] = "AHRS/data_Madgwick";
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

MeasurementData data;
AHRSData MadgwickResult;
AHRSData MahonyResult;
AHRSData KalmanResult;
AHRSData ComplementarFilterResult;

void update_measurements(MeasurementData &data){
  int16_t IMU_data[7];
  IMU.readData(IMU_data);
  data.accelerometer_x = IMU_data[1] * IMU.getAres(AFS_8G);
  data.accelerometer_y = IMU_data[2] * IMU.getAres(AFS_8G);
  data.accelerometer_z = IMU_data[3] * IMU.getAres(AFS_8G);

  data.gyroscope_x = IMU_data[4] * IMU.getAres(AFS_8G);
  data.gyroscope_y = IMU_data[5] * IMU.getAres(AFS_8G);
  data.gyroscope_z = IMU_data[6] * IMU.getAres(AFS_8G);

  data.mag_x = MAG.getMeasurementX();
  data.mag_y = MAG.getMeasurementY();
  data.mag_z = MAG.getMeasurementZ();
}

int16_t IMU_data[7];
uint32_t currentX = 0;
uint32_t currentY = 0;
uint32_t currentZ = 0;

void setup() {
  Wire.begin(4,5);
  IMU.init(AFS_8G, GFS_2000DPS, AODR_200Hz, GODR_200Hz);
  MAG.begin();
  MAG.softReset();
  Serial.begin(115200);

  MadgwickFilter.begin(200);
  MahonyFilter.begin(200);

	// connect to wifi
	Serial.print("Connecting to WiFi");
	WiFi.begin(WIFI_SSID, WIFI_PSWD);
	while (WiFi.status() != WL_CONNECTED) {
		delay(1000);
		Serial.print(".");
  	}

	Serial.println("Connecting to MQTT host");
	// mqtt.setUsernamePassword(
	// 	MQTT_USER,
	// 	MQTT_PSWD
	// );
	if (!mqtt.connect(MQTT_BROKER_IP, MQTT_BROKER_PORT)) {
		Serial.print("MQTT connection failed! Error code = ");
		Serial.println(mqtt.connectError());
		while (1);
  	}
	Serial.println("MQTT connected");

}
 
void loop() {

  MadgwickFilter.update(data.gyroscope_x, data.gyroscope_y, data.gyroscope_z, data.accelerometer_x, data.accelerometer_y, data.accelerometer_z, data.mag_x, data.mag_y, data.mag_z);
  MadgwickResult.roll = MadgwickFilter.getRoll() * RAD_TO_DEG;
  MadgwickResult.pitch = MadgwickFilter.getPitch() * RAD_TO_DEG;
  MadgwickResult.yaw = MadgwickFilter.getYaw() * RAD_TO_DEG;

  MahonyFilter.update(data.gyroscope_x, data.gyroscope_y, data.gyroscope_z, data.accelerometer_x, data.accelerometer_y, data.accelerometer_z, data.mag_x, data.mag_y, data.mag_z);
  MahonyResult.roll = MahonyFilter.getRoll() * RAD_TO_DEG;
  MahonyResult.pitch = MahonyFilter.getPitch() * RAD_TO_DEG;
  MahonyResult.yaw = MahonyFilter.getYaw() * RAD_TO_DEG;

  String data_to_print;
  data_to_print = String(MadgwickResult.roll)+';'+String(MadgwickResult.pitch)+';'+String(MadgwickResult.yaw);
  sendMqttMsg(data_to_print, topic_dataMadgwick, mqtt, debug);

  data_to_print = String(MahonyResult.roll)+';'+String(MahonyResult.pitch)+';'+String(MahonyResult.yaw);
  sendMqttMsg(data_to_print, topic_dataMahony, mqtt, debug);
  
  delay(100);
}