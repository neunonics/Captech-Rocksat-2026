// -- LIBRARIES -- //
#ifndef FSW_H
#define FSW_H

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

#define HEARTBEAT_INTERVAL 1 // Flip heartbeat LED every 1 second
#define SD_SAVE_INTERVAL 1 // Save to SD card every 1 second (TIMERS OPERATE IN SECONDS, SO THIS IS 1 SECOND INTERVAL)
#define TE2_CHECK_INTERVAL 1 // Check TE-2 signal every 1 second

#define COMM_SEND_INTERVAL 30 // Send message via COMMS every 30 seconds
#define COMM_TIMEOUT 30 // Timeout for COMMS transmission in seconds (RB9704 README: use timeouts of several minutes, not seconds)

#define TE_2_TRIGGER_THRESHOLD 2 // Number of times TE-2 signal must be read high before transitioning to Science Mode

#define DEBUG_SERIAL Serial // USB Debug Serial
#define DEBUG_SERIAL_BAUD 9600// USB Debus Serial Baud Rate

#define PRIORITY_MAX_ROWS 25 // Cap on entries kept in the ORIN priority queue
#define PRIORITY_TMP_FILE "ptmp.txt" // Working file used while rewriting the priority file

struct FSW {
    // -- FSW STATUS -- //
    bool LAUNCH; // TE-2 has not been triggered (Launch Mode)
    bool SCIENCE; // TE-2 has been triggered (Science Mode)
    bool ATTITUDE_RDY; // BNO055s have been initialized and are ready to log attitude data
    bool RTC_RDY; // RTC has been initialized and is ready to provide time data
    bool SD_RDY; // SD Card has been initialized and is ready to log data
    bool HRTBT; // Heartbeat LED state (true = on, false = off)
    uint8_t TE2_TRIGGERS; // Number of times TE2 pin was read high

    // -- FSW FILES -- //
    File currentFile; // Current File for SD Card
    String fswToSave; // FSW data to save to SD Card in one line
    String epdsToSave; // EPDS data to save to SD Card in one line
    String histogramAToSave; // Histogram data to save to SD Card in one line
    String histogramBToSave; // Histogram data to save to SD Card in one line
    String AIToSave; // AI data to save to SD Card in one line
    String combinedHistogram; // Combined Histogram to Transmit
    char fileName[13]; // String to carry the name of the file to save to on the SD Card
    char priorityFileName[13]; // SD filename for the ORIN priority queue (p<last7>.txt)

    // -- FSW SENSORS -- //
    Adafruit_BNO055 BNO055_A;
    Adafruit_BNO055 BNO055_B;

    imu::Vector<3> euler_A; // Euler angles from BNO055 A
    imu::Vector<3> linAcc_A; // Linear acceleration from BNO055 A
    imu::Quaternion quat_A; // Quaternion from BNO055 A
    imu::Vector<3> mag_A; // Magnetometer data from BNO055 A
    imu::Vector<3> euler_B; // Euler angles from BNO055 B
    imu::Vector<3> linAcc_B; // Linear acceleration from BNO055 B
    imu::Quaternion quat_B; // Quaternion from BNO055 B
    imu::Vector<3> mag_B; // Magnetometer data from BNO055 B

    // -- FSW TIMERS -- //
    unsigned long missionStartTime; // Start Time of Mission (s)
    unsigned long currentMissionTime; // Current Time of Mission (s)
    unsigned long lastHeartbeatTime; // Last Time Attitude Data was Logged (s)
    unsigned long lastPredictionSave; // Last Time Prediction was Logged (s)
    unsigned long lastSDCardSave; // Last Time SD Card Data was Saved (s)
    unsigned long lastTE2CheckTime; // Last Time TE2 pin was checked (s)
    unsigned long lastTransmit; // Last Time Attempted to Transmit (s)
};


// -- INIT FUNCTIONS -- //
void initPins();
void initFSWStatus(FSW &fsw);
bool initSDCard(FSW &fsw);
void deinitSDCard(FSW &fsw);
bool initRTC(FSW &fsw);
time_t getTeensy3Time();
bool initBNO055(FSW &fsw);
void initTimers(FSW &fsw);

// -- LOGGING FUNCTIONS -- //
void readAttitude(FSW &fsw);
void readEPDS(EPDS &epds, FSW &fsw);
void logData(FSW &fsw);
bool getLatestPrediction(FSW &fsw, String &predictionOut, String &combinedHistogramOut);

// -- PRIORITY QUEUE (ORIN predictions) -- //
float Priority_ParseTopConfidence(const String &orinLine);
bool  Priority_Insert(FSW &fsw, const String &orinLine, const String &combinedHist);
bool  Priority_PeekTop(FSW &fsw, String &orinLineOut, String &histOut);
bool  Priority_DeleteTop(FSW &fsw);

#endif