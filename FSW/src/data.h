#include <ArduinoJson.h>
#include <SD.h>
#include <STM32RTC.h>

#define NUM_BINS 1024 // Adjust this based on your detector's resolution

struct FSW_DATETIME{
    u_int8_t day;
    u_int8_t hour;
    u_int8_t second;
    u_int8_t month;
    u_int8_t minute;
    u_int8_t year;
    uint32_t subseconds;
};

struct FSW_GAMMA_DATA {
    FSW_DATETIME datetime;
    uint32_t spectrumID;
    uint16_t bins[NUM_BINS];
    bool valid; 
};

struct FSW_STATUS{
    bool enable_spectrometer;
    bool enable_iridium;
    bool led_comms;
    bool led_error;
    bool led_sd;
    bool led_hrtbt;
    int error_code;
    bool is_armed;
    bool is_te2;

    bool spect_1_connected;
    bool spect_2_connected;
    bool orin_connected;
    bool comms_connected;
    bool bno1_connected;
    bool bno2_connected;

};

struct FSW_IMU_EULER{
    float roll;
    float pitch;
    float yaw;
};

struct FSW_IMU_XYZ{
    float x;
    float y;
    float z;
};

struct FSW_IMU_QUAT{
    float q1;
    float q2;
    float q3;
    float q4;
};

struct FSW_IMU{
    FSW_IMU_EULER euler;
    FSW_IMU_XYZ bfield;
    FSW_IMU_XYZ gravity;
    FSW_IMU_QUAT quaternion;
};

struct FSW_ADS{
    FSW_IMU imu1;
    FSW_IMU imu2;
};

struct FSW_PREDICTION{
    int priority;
    int spectrumID;
    String prediction;
    float probability;
};

struct FSW_POWER{
    float voltage;
    float current;
    float power;
};

struct FSW_EDS{
    FSW_POWER twelveV;
    FSW_POWER fiveV;
    FSW_POWER threeV;

};

struct FSW_SYSTEM{
    FSW_EDS epds;
    FSW_IMU ads;
    FSW_STATUS status;
};

void saveData(int round, FSW_SYSTEM sys, FSW_DATETIME);
FSW_GAMMA_DATA fetchSpectrum(int, HardwareSerial, FSW_DATETIME);