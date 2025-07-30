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
#include <Magisterka_inferencing.h>

bool debug = true;

WiFiClient wifiClient;
MqttClient mqtt(wifiClient);

ICM40627 IMU;
SFE_MMC5983MA MAG;
Madgwick MadgwickFilter;
Mahony MahonyFilter;

char topic_dataMadgwick[] = "AHRS/data_Madgwick";
char topic_rawdata[] = "rawdata";
char topic_state[] = "state_changes";
char topic_gesture[] = "gestures";

void sendMqttMsg(String to_print, String topic, MqttClient& mqtt, bool debug) {
    mqtt.beginMessage(topic);
    mqtt.print(to_print);
    mqtt.endMessage();
    if(debug) {
        // Serial.println(String(to_print + " : to : " + String(topic)));
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
    float accel_m;
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

const int num_sections = 10;
const float roll_range[2] = {-180.0, 180.0};
const float pitch_range[2] = {-90.0, 90.0};
const float yaw_range[2] = {-180.0, 180.0};
const float accel_range[2] = {0., 2.};
float max_accel = 0;
int hysteresis_margin = 4;

int prev_roll_state = -1, prev_pitch_state = -1, prev_yaw_state = -1, prev_accel_state = -1;

const int window_size = 5; // Size of the moving window
const int jump_size = 1; // Jump size for the moving window

int state_window[window_size * 4]; // Buffer to hold the state changes for window_size

int window_index = 0;

void filterData(MeasurementData &incommingData, MeasurementData &filteredData) {
    filters[0]->calculate_output(incommingData.accelerometer_x, &filteredData.accelerometer_x);
    filters[1]->calculate_output(incommingData.accelerometer_y, &filteredData.accelerometer_y);
    filters[2]->calculate_output(incommingData.accelerometer_z, &filteredData.accelerometer_z);
    filters[3]->calculate_output(incommingData.gyroscope_x, &filteredData.gyroscope_x);
    filters[4]->calculate_output(incommingData.gyroscope_y, &filteredData.gyroscope_y);
    filters[5]->calculate_output(incommingData.gyroscope_z, &filteredData.gyroscope_z);
}

// Define calibration parameters
float A[3][3] = {
    {1.104079, -0.008604, 0.001812},
    {-0.008604, 1.192882, -0.058367},
    {0.001812, -0.058367, 1.180375}
};
float b[3] = {-13.137503, 24.333966, 31.479244};

MeasurementData update_measurements() {
    MeasurementData data;
    int16_t IMU_data[7];
    IMU.readData(IMU_data);
    data.accelerometer_x = IMU_data[1] * IMU.getAres(AFS_8G);
    data.accelerometer_y = IMU_data[2] * IMU.getAres(AFS_8G);
    data.accelerometer_z = IMU_data[3] * IMU.getAres(AFS_8G);

    data.gyroscope_x = IMU_data[4] * IMU.getAres(GFS_2000DPS);
    data.gyroscope_y = IMU_data[5] * IMU.getAres(GFS_2000DPS);
    data.gyroscope_z = IMU_data[6] * IMU.getAres(GFS_2000DPS);

    data.accel_m = sqrt(pow(data.accelerometer_x,2) + pow(data.accelerometer_y,2) + pow(data.accelerometer_z,2));

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

    MeasurementData temp = data;
    data.accelerometer_x = temp.accelerometer_y;
    data.accelerometer_y = temp.accelerometer_x;
    data.accelerometer_z = -temp.accelerometer_z;

    data.gyroscope_x = temp.gyroscope_y;
    data.gyroscope_y = temp.gyroscope_x;
    data.gyroscope_z = -temp.gyroscope_z;

    data.mag_x = -temp.mag_x;

    MeasurementData temp2;
    filterData(data, temp2);
    data.gyroscope_x = temp.gyroscope_x;
    data.gyroscope_y = temp.gyroscope_y;
    data.gyroscope_z = temp.gyroscope_z;
    data.accelerometer_x = temp.accelerometer_x;
    data.accelerometer_y = temp.accelerometer_y;
    data.accelerometer_z = temp.accelerometer_z;

    return data;
}

void IMU_calib() {
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
    for(int i = 0; i < 50; i++) {
        IMU.readData(IMU_data);
        gyroscope_x = IMU_data[4] * IMU.getAres(GFS_2000DPS);
        gyroscope_y = IMU_data[5] * IMU.getAres(GFS_2000DPS);
        gyroscope_z = IMU_data[6] * IMU.getAres(GFS_2000DPS);
        sumx += gyroscope_x;
        sumy += gyroscope_y;
        sumz += gyroscope_z;
        delay(5);
    }
    offsetX = sumx / 50;
    offsetY = sumy / 50;
    offsetZ = sumz / 50;
    IMU.setOffsets(0., 0., 0., offsetX, offsetY, offsetZ);
}

int determine_state(float value, float range_min, float range_max, int num_sections, int hysteresis_margin, int prev_state) {
    float section_width = (range_max - range_min) / float(num_sections);

    if(prev_state != -1){

        if(prev_state == 0 && (value < range_max - hysteresis_margin && value > range_max - section_width)){
            return num_sections - 1;
        } else if (prev_state == num_sections - 1 && (value > range_min + hysteresis_margin && value < range_min + section_width)){
            return 0;
        }

        float lower_bound = range_min + prev_state * section_width;
        float upper_bound = lower_bound + section_width;

        if(value < lower_bound - hysteresis_margin && value > lower_bound - section_width){
            return max(prev_state - 1, 0);
        } else if (value >= upper_bound + hysteresis_margin && value < upper_bound + section_width){
            return min(prev_state + 1, num_sections - 1);
        } else {
            return prev_state;
        }

    } else {
        if (value < range_min) {
            return 0;
        } else if (value >= range_max) {
            return num_sections - 1;
        } else {
            return int((value - range_min) / section_width);
        }
    }
}

int determine_accel_state(float max_accel, float range_min, float range_max, int num_sections){
    float accel_min = range_min;
    float accel_max = range_max;
    float section_width = (accel_max - accel_min) / float(num_sections);
    if(max_accel < accel_min){
        return 0;
    } else if (max_accel >= accel_max){
        return num_sections - 1;
    } else {
        return ((max_accel - accel_min) / section_width);
    }
}

void setup() {
    Wire.begin(5, 6);
    Serial.begin(115200);
    IMU.init(AFS_8G, GFS_2000DPS, AODR_1000Hz, GODR_1000Hz);
    IMU_calib();
    for (int filter_id = 0; filter_id < 6; filter_id++) {
        filters[filter_id] = new LowPassFilter(a_coeff, b_coeff);
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

    LoopTime = 1000000 / 100;
}

unsigned long currentTime;
unsigned long dt;
unsigned long prevTime = 0;
MeasurementData data;

void update_state_window(int roll_state, int pitch_state, int yaw_state, int accel_state, int* window, int& index, int size) {
    if (index >= size * 4) {
        memmove(window, window + jump_size * 4, (size - jump_size) * 4 * sizeof(int));
        index -= jump_size * 4;
    }
    window[index++] = roll_state;
    window[index++] = pitch_state;
    window[index++] = yaw_state;
    window[index++] = accel_state;
}

void predict_gesture(int* window, int index, int size, float& entropy, String& highest_prediction) {
    if (index < size * 4) return;

    // Pad the smaller windows with zeros
    int padded_window[window_size * 4] = {0};
    memcpy(padded_window, window, index * sizeof(int));

    signal_t signal;
    signal.total_length = window_size * 4;
    signal.get_data = [padded_window](size_t offset, size_t length, float *out_ptr) -> int {
        for (size_t i = 0; i < length; i++) {
            out_ptr[i] = (float)padded_window[offset + i];
        }
        return 0;
    };

    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
    if (res != EI_IMPULSE_OK) {
        Serial.println("ERROR: Failed to run classifier");
        return;
    }

    size_t max_index = 0;
    float max_value = result.classification[0].value;
    if(result.classification[0].value != 0.) entropy += result.classification[0].value * log(result.classification[0].value);
    for (size_t ix = 1; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if(result.classification[ix].value != 0.) entropy += result.classification[ix].value * log(result.classification[ix].value);
        if (result.classification[ix].value > max_value) {
            max_value = result.classification[ix].value;
            max_index = ix;
        }
    }
    entropy = -entropy;
    // Serial.println(entropy);
    if (entropy < 0.05) {
        highest_prediction = String(result.classification[max_index].label) + ": " + String(entropy);
    } else {
        highest_prediction = "uncertain";
    }
}

void loop() {
    currentTime = micros();
    dt = currentTime - prevTime;
    if (dt >= LoopTime) {
        prevTime = currentTime;

        data = update_measurements();
        MadgwickFilter.update(data.gyroscope_x, data.gyroscope_y, data.gyroscope_z, data.accelerometer_x, data.accelerometer_y, data.accelerometer_z, data.mag_x, data.mag_y, data.mag_z);

        MadgwickResult.roll = MadgwickFilter.getRoll();
        MadgwickResult.pitch = MadgwickFilter.getPitch();
        MadgwickResult.yaw = MadgwickFilter.getYaw();

        int roll_state = determine_state(MadgwickResult.roll, roll_range[0], roll_range[1], num_sections, hysteresis_margin, prev_roll_state);
        int pitch_state = determine_state(MadgwickResult.pitch, pitch_range[0], pitch_range[1], num_sections, hysteresis_margin, prev_pitch_state);
        int yaw_state = determine_state(MadgwickResult.yaw, yaw_range[0], yaw_range[1], num_sections, hysteresis_margin, prev_yaw_state);
        if (data.accel_m > max_accel){
            max_accel = data.accel_m;
            // Serial.println(max_accel);
        }

        if (roll_state != prev_roll_state || pitch_state != prev_pitch_state || yaw_state != prev_yaw_state) {
            int accel_state = determine_accel_state(max_accel, accel_range[0], accel_range[1], num_sections);
            String state_data = String(roll_state) + ',' + String(pitch_state) + ',' + String(yaw_state) + ',' + String(accel_state);
            sendMqttMsg(state_data, topic_state, mqtt, debug);

            update_state_window(roll_state, pitch_state, yaw_state, accel_state, state_window, window_index, window_size);

            prev_roll_state = roll_state;
            prev_pitch_state = pitch_state;
            prev_yaw_state = yaw_state;
            max_accel = 0;
            float entropy = 0.;
            String highest_prediction;

            predict_gesture(state_window, window_index, window_size, entropy, highest_prediction);

            if (entropy < 0.05) {
                sendMqttMsg(highest_prediction, topic_gesture, mqtt, debug);
            }
        }

        String data_to_print = String(MadgwickResult.roll) + ';' + String(MadgwickResult.pitch) + ';' + String(MadgwickResult.yaw);
        sendMqttMsg(data_to_print, topic_dataMadgwick, mqtt, debug);

        data_to_print = String(data.accelerometer_x) + ';' + String(data.accelerometer_y) + ';' + String(data.accelerometer_z) + ';' + String(data.gyroscope_x) + ';' + String(data.gyroscope_y) + ';' + String(data.gyroscope_z) + ';' + String(data.mag_x) + ';' + String(data.mag_y) + ';' + String(data.mag_z);
        sendMqttMsg(data_to_print, topic_rawdata, mqtt, debug);
    }
}
