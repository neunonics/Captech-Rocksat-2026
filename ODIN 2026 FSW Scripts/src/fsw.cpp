#include <TimeLib.h> // RTC LIBRARY
#include "fsw.h" // FSW HEADER

// -- INIT FUNCTIONS -- //

// Function to initialize FSW status variables
void initFSWStatus(FSW &fsw) {
  fsw.LAUNCH = true; // TE-2 has not been triggered (Launch Mode)
  fsw.SCIENCE = false; // TE-2 has been triggered (Science Mode)
  fsw.ATTITUDE_RDY = false; // BNO055s have been initialized and are ready to log attitude data
  fsw.RTC_RDY = false; // RTC has been initialized and is ready to provide time data
  fsw.SD_RDY = false; // SD Card has been initialized and is ready to log data
}

bool initSDCard(FSW &fsw) {
  if (!SD.begin(BUILTIN_SDCARD)) {
    return false; // SD Card initialization failed
  }
  fsw.SD_RDY = true; // Set SD Card ready flag to true after successful initialization
  return true;
}

// Function to initialize RTC & Mission Start Time
bool initRTC(FSW &fsw) {
  setSyncProvider(getTeensy3Time); // Set Teensy RTC as the time provider
  delay(100); // Short delay to ensure RTC is ready
  if (timeStatus() != timeSet) {
    return false; // Unable to sync with the RTC
  }
  fsw.missionStartTime = now(); // Record the mission start time
  Serial.print("Mission Start Time: ");
  Serial.println(fsw.missionStartTime);
  fsw.RTC_RDY = true; // Set RTC ready flag to true after successful initialization
  return true;
}

// Function to initialize BNO055 sensors
bool initBNO055(FSW &fsw) {
  // Initialize BNO055 A
  fsw.BNO055_A = Adafruit_BNO055(BNO055_SENSOR_ID, BNO055_ADDRESS_A);
  if (!fsw.BNO055_A.begin()) {
    return false; // FAILED BNO055 A INITIALIZATION
  }
  // Initialize BNO055 B
  fsw.BNO055_B = Adafruit_BNO055(BNO055_SENSOR_ID, BNO055_ADDRESS_B);
  if (!fsw.BNO055_B.begin()) {
    return false; // FAILED BNO055 B INITIALIZATION
  }

  fsw.ATTITUDE_RDY = true; // Set BNO055 ready flag to true after successful initialization
  return true;
}

void initPins() {
  //Initalize Pins for Output
  pinMode(LED_PWR, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_COMM, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);
  pinMode(LED_SDACTIVE, OUTPUT);

  //Initalize Pins for Input
  pinMode(TE2_SIGNAL, INPUT);
  return;
}

// -- LOGGING FUNCTIONS -- //

