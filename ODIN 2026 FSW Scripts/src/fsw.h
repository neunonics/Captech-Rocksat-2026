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

// -- DEFINITIONS -- //
#define BNO055_SENSOR_ID 55
#define BNO055_ADDRESS_A 0x28
#define BNO055_ADDRESS_B 0x29

struct FSW {
    // -- FSW STATUS -- //
    bool LAUNCH; // TE-2 has not been triggered (Launch Mode)
    bool SCIENCE; // TE-2 has been triggered (Science Mode)
    bool ATTITUDE_RDY; // BNO055s have been initialized and are ready to log attitude data
    bool RTC_RDY; // RTC has been initialized and is ready to provide time data
    bool SD_RDY; // SD Card has been initialized and is ready to log data

    // -- FSW FILES -- //
    File launchFile; // Launch File for SD Card
    File scienceFile; // Science File for SD Card
    String lineToSave; // String to carry next line to save to the SD Card

    // -- FSW SENSORS -- //
    Adafruit_BNO055 BNO055_A;
    Adafruit_BNO055 BNO055_B;

    // -- FSW TIMERS -- //
    unsigned long missionStartTime; // Start Time of Mission (s)
    unsigned long currentMissionTime; // Current Time of Mission (s)
};


// -- INIT FUNCTIONS -- //
void initPins();
void initFSWStatus(FSW &fsw);
bool initSDCard(FSW &fsw);
bool initRTC(FSW &fsw);
bool initBNO055(FSW &fsw);

// -- LOGGING FUNCTIONS -- //
String readAttitude(FSW &fsw);
unsigned long deltaTime(unsigned long currentTime, unsigned long startTime);