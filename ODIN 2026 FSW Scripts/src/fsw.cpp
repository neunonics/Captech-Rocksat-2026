#include <TimeLib.h> // RTC LIBRARY
#include "fsw.h" // FSW HEADER

// Function to initialize FSW status variables
void initFSWStatus(FSW &fsw) {
  fsw.LAUNCH = true; // TE-2 has not been triggered (Launch Mode)
  fsw.SCIENCE = false; // TE-2 has been triggered (Science Mode)
  fsw.ATTITUDE_RDY = false; // BNO055s have been initialized and are ready to log attitude data
  fsw.RTC_RDY = false; // RTC has been initialized and is ready to provide time data
}

// Function to initialize RTC & Mission Start Time
bool initTime(FSW &fsw) {
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

time_t getTeensy3Time() {
  return Teensy3Clock.get();
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