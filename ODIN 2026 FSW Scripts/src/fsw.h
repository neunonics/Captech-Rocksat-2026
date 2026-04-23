// -- LIBRARIES -- //
#include <Arduino.h> // STANDARD ARDUINO LIBRARY
#include <Wire.h> // WIRE LIBRARY

#include <SD.h> // SD CARD
#include <SPI.h> // SD CARD DATA PROTOCOL

#include <TimeLib.h> // RTC LIBRARY

#include <Adafruit_Sensor.h> // BNO055 LIBRARY
#include <Adafruit_BNO055.h> // BNO055 LIBRARY
#include <utility/imumaths.h> // BNO055 LIBRARY

#include "pins.h"

#include "epds.h" // EPDS HEADER

// -- DEFINITIONS -- //
#define BNO055_SENSOR_ID 55
#define BNO055_ADDRESS_A 0x28
#define BNO055_ADDRESS_B 0x29

#define HEARTBEAT_INTERVAL 1 // Flip heartbeat LED every 1 second
#define SD_SAVE_INTERVAL 1 // Save to SD card every 1 second (TIMERS OPERATE IN SECONDS, SO THIS IS 1 SECOND INTERVAL)

struct FSW {
    // -- FSW STATUS -- //
    bool LAUNCH; // TE-2 has not been triggered (Launch Mode)
    bool SCIENCE; // TE-2 has been triggered (Science Mode)
    bool ATTITUDE_RDY; // BNO055s have been initialized and are ready to log attitude data
    bool RTC_RDY; // RTC has been initialized and is ready to provide time data
    bool SD_RDY; // SD Card has been initialized and is ready to log data

    // -- FSW FILES -- //
    File currentFile; // Current File for SD Card
    String fswToSave; // FSW data to save to SD Card in one line
    String epdsToSave; // EPDS data to save to SD Card in one line
    String histogramAToSave; // Histogram data to save to SD Card in one line
    String histogramBToSave; // Histogram data to save to SD Card in one line
    String AIToSave; // AI data to save to SD Card in one line
    char fileName[13]; // String to carry the name of the file to save to on the SD Card

    // -- FSW SENSORS -- //
    Adafruit_BNO055 BNO055_A;
    Adafruit_BNO055 BNO055_B;

    imu::Vector<3> euler_A; // Euler angles from BNO055 A
    imu::Vector<3> linAcc_A; // Linear acceleration from BNO055 A
    imu::Quaternion quat_A; // Quaternion from BNO055 A
    int8_t temp_A; // Temperature from BNO055 A
    imu::Vector<3> euler_B; // Euler angles from BNO055 B
    imu::Vector<3> linAcc_B; // Linear acceleration from BNO055 B
    imu::Quaternion quat_B; // Quaternion from BNO055 B
    int8_t temp_B; // Temperature from BNO055 B

    // -- FSW TIMERS -- //
    unsigned long missionStartTime; // Start Time of Mission (s)
    unsigned long currentMissionTime; // Current Time of Mission (s)
    unsigned long lastHeartbeatTime; // Last Time Attitude Data was Logged (s)
    unsigned long lastSDCardSave; // Last Time SD Card Data was Saved (s)
};


// -- INIT FUNCTIONS -- //
void initPins();
void initFSWStatus(FSW &fsw);
bool initSDCard(FSW &fsw);
void deinitSDCard(FSW &fsw);
bool initRTC(FSW &fsw);
bool initBNO055(FSW &fsw);
void initTimers(FSW &fsw);

// -- LOGGING FUNCTIONS -- //
void readAttitude(FSW &fsw);
void readEPDS(EPDS &epds, FSW &fsw);
void logData(FSW &fsw);