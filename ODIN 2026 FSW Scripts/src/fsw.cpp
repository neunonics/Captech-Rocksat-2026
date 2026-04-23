#include <TimeLib.h> // RTC LIBRARY
#include <SD.h>
#include <SPI.h>

#include "fsw.h" // FSW HEADER
#include "epds.h" // EPDS HEADER

// -- INIT FUNCTIONS -- //

// Function to initialize FSW status variables
void initFSWStatus(FSW &fsw) {
  fsw.LAUNCH = true; // TE-2 has not been triggered (Launch Mode)
  fsw.SCIENCE = false; // TE-2 has been triggered (Science Mode)
  fsw.ATTITUDE_RDY = false; // BNO055s have been initialized and are ready to log attitude data
  fsw.RTC_RDY = false; // RTC has been initialized and is ready to provide time data
  fsw.SD_RDY = false; // SD Card has been initialized and is ready to log data
  fsw.HRTBT = false; // Heartbeat LED state (true = on, false = off)
  fsw.TE2_TRIGGERED = false; // TE2 has been triggered 
}

bool initSDCard(FSW &fsw) {
  if (!SD.begin(BUILTIN_SDCARD)) {
    return false; // SD Card initialization failed
  }
  digitalWrite(LED_SDACTIVE, HIGH); // Ensure SD activity LED is off after initialization
  fsw.SD_RDY = true; // Set SD Card ready flag to true after successful initialization
  return true;
}

void deinitSDCard(FSW &fsw) {
  fsw.SD_RDY = false; // Set SD Card ready flag to false after deinitialization
  digitalWrite(LED_SDACTIVE, LOW); // Turn off SD activity LED
  return;
}

// Function to initialize RTC & Mission Start Time
bool initRTC(FSW &fsw) {
  setSyncProvider([]() -> time_t { return (time_t)rtc_get(); }); // Set Teensy RTC as the time provider
  delay(100); // Short delay to ensure RTC is ready
  if (timeStatus() != timeSet) {
    return false; // Unable to sync with the RTC
  }
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
    Serial.println("[FSW] ERROR: Failed to initialize BNO055 A!");
    return false; // FAILED BNO055 A INITIALIZATION
  }
  // Initialize BNO055 B
  fsw.BNO055_B = Adafruit_BNO055(BNO055_SENSOR_ID, BNO055_ADDRESS_B);
  if (!fsw.BNO055_B.begin()) {
    Serial.println("[FSW] ERROR: Failed to initialize BNO055 B!");
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
  pinMode(LED_ERROR, OUTPUT);
  pinMode(LED_SDACTIVE, OUTPUT);
  pinMode(COMM_EN, OUTPUT);

  //Initalize Pins for Input
  pinMode(TE2_SIGNAL, INPUT);
  pinMode(COMM_BTD, INPUT);
  return;
}

void initTimers(FSW &fsw) {
  fsw.missionStartTime = now(); // Start Time of Mission (s)
  fsw.currentMissionTime = now(); // Current Time of Mission (s)
  fsw.lastHeartbeatTime = now(); // Last Time Attitude Data was Logged (s)
  fsw.lastSDCardSave = now(); // Last Time SD Card Data was Saved (s)
}

// -- LOGGING FUNCTIONS -- //

void readAttitude(FSW &fsw) {
  if(!fsw.ATTITUDE_RDY) {
    Serial.println("[FSW] ERROR: Cannot log attitude until BNO055 is ready.");
    if (!initBNO055(fsw)) {
      Serial.println("[FSW] ERROR: Failed to re-initialize BNO055!");
    } else {
      Serial.println("[FSW] Successfully re-initialized BNO055!");
    }
  } else {
    // Read all attitude data from BNO055 A and BNO055 B and store in FSW struct
    fsw.euler_A = fsw.BNO055_A.getVector(Adafruit_BNO055::VECTOR_EULER); // Euler angles from BNO055 A
    fsw.linAcc_A = fsw.BNO055_A.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); // Linear acceleration from BNO055 A
    fsw.quat_A = fsw.BNO055_A.getQuat(); // Quaternion from BNO055 A
    fsw.temp_A = fsw.BNO055_A.getTemp(); // Temperature from BNO055 A

    fsw.euler_B = fsw.BNO055_B.getVector(Adafruit_BNO055::VECTOR_EULER); // Euler angles from BNO055 B
    fsw.linAcc_B = fsw.BNO055_B.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); // Linear acceleration from BNO055 B
    fsw.quat_B = fsw.BNO055_B.getQuat(); // Quaternion from BNO055 B
    fsw.temp_B = fsw.BNO055_B.getTemp(); // Temperature from BNO055 B

    // Print attitude data to Serial Monitor for debugging
    Serial.print("[FSW] -- BNO055 A Data --");
    Serial.print("Euler: ");
    Serial.print(fsw.euler_A.x(), 3);
    Serial.print(", ");
    Serial.print(fsw.euler_A.y(), 3);
    Serial.print(", ");
    Serial.println(fsw.euler_A.z(), 3);
    Serial.print("Lin Acc: ");
    Serial.print(fsw.linAcc_A.x(), 3);
    Serial.print(", ");
    Serial.print(fsw.linAcc_A.y(), 3);
    Serial.print(", ");
    Serial.println(fsw.linAcc_A.z(), 3);
    Serial.print("Quaternion: ");
    Serial.print(fsw.quat_A.x(), 3);
    Serial.print(", ");
    Serial.print(fsw.quat_A.y(), 3);
    Serial.print(", ");
    Serial.print(fsw.quat_A.z(), 3);
    Serial.print(", ");
    Serial.println(fsw.quat_A.w(), 3);
    Serial.print("Temperature: ");
    Serial.println(fsw.temp_A, 3);

    Serial.print("[FSW] -- BNO055 B Data --");
    Serial.print("Euler: ");
    Serial.print(fsw.euler_B.x(), 3);
    Serial.print(", ");
    Serial.print(fsw.euler_B.y(), 3);
    Serial.print(", ");
    Serial.println(fsw.euler_B.z(), 3);
    Serial.print("Lin Acc: ");
    Serial.print(fsw.linAcc_B.x(), 3);
    Serial.print(", ");
    Serial.print(fsw.linAcc_B.y(), 3);
    Serial.print(", ");
    Serial.println(fsw.linAcc_B.z(), 3);
    Serial.print("Quaternion: ");
    Serial.print(fsw.quat_B.x(), 3);
    Serial.print(", ");
    Serial.print(fsw.quat_B.y(), 3);
    Serial.print(", ");
    Serial.print(fsw.quat_B.z(), 3);
    Serial.print(", ");
    Serial.println(fsw.quat_B.w(), 3);
    Serial.print("Temperature: ");
    Serial.println(fsw.temp_B, 3);
    fsw.fswToSave += String(fsw.euler_A.x(), 3) + ";" + String(fsw.euler_A.y(), 3) + ";" + String(fsw.euler_A.z(), 3) + ";" + // Log BNO055 A Euler angles
                      String(fsw.linAcc_A.x(), 3) + ";" + String(fsw.linAcc_A.y(), 3) + ";" + String(fsw.linAcc_A.z(), 3) + ";" + // Log BNO055 A Linear Acceleration
                      String(fsw.quat_A.x(), 3) + ";" + String(fsw.quat_A.y(), 3) + ";" + String(fsw.quat_A.z(), 3) + ";" + String(fsw.quat_A.w(), 3) + ";" + // Log BNO055 A Quaternion
                      String(fsw.temp_A, 3) + ";"; // Log BNO055 A Temperature
    fsw.fswToSave += String(fsw.euler_B.x(), 3) + ";" + String(fsw.euler_B.y(), 3) + ";" + String(fsw.euler_B.z(), 3) + ";" + // Log BNO055 B Euler angles
                      String(fsw.linAcc_B.x(), 3) + ";" + String(fsw.linAcc_B.y(), 3) + ";" + String(fsw.linAcc_B.z(), 3) + ";" + // Log BNO055 B Linear Acceleration
                      String(fsw.quat_B.x(), 3) + ";" + String(fsw.quat_B.y(), 3) + ";" + String(fsw.quat_B.z(), 3) + ";" + String(fsw.quat_B.w(), 3) + ";" + // Log BNO055 B Quaternion
                      String(fsw.temp_B, 3) + ";"; // Log BNO055 B Temperature
  }
}

void readEPDS(EPDS &epds, FSW &fsw) {
  if(!epds.initialized) {
    Serial.println("[EPDS] ERROR: Cannot log EPDS data until EPDS is initialized.");
    if (EPDS_init(epds)) {
      Serial.println("[EPDS] EPDS initialization successful on retry!");
    } else {
      Serial.println("[EPDS] EPDS initialization failed again on retry!");
    }
  } else {
    EPDS_readAll(epds);

    // Use voltage values directly, do something with them as needed (e.g., log, check thresholds, etc.)
    float rkt  = epds.RKT_V;
    float v12  = epds.V12_V;
    float v5   = epds.V5_V;
    float v3v3 = epds.V3V3_V;

    // for debugging, print the voltage values to the serial monitor in one line
    Serial.print("[EPDS] RKT Voltage: ");
    Serial.print(rkt, 3);
    Serial.print(" | 12V Bus: ");
    Serial.print(v12, 3);
    Serial.print(" | 5V Bus: ");
    Serial.print(v5, 3);
    Serial.print(" | 3V3 Bus: ");
    Serial.println(v3v3, 3);

    fsw.epdsToSave += String(rkt, 3) + ";" + String(v12, 3) + ";" + String(v5, 3) + ";" + String(v3v3, 3) + ";"; // Log EPDS voltages
  }
}

void logData(FSW &fsw) {
  // -- Save to SD Card -- //
  if ((fsw.currentMissionTime - fsw.lastSDCardSave) >= SD_SAVE_INTERVAL) {
    if(!fsw.SD_RDY) {
      Serial.println("[FSW] ERROR: Cannot log data until SD card is ready.");
      if(!initSDCard(fsw)) {
        Serial.println("[FSW] ERROR: Attempted SD card initialization and failed!");
      } else {
        Serial.println("[FSW] SD card initialization successful!");
      }
    } else {
      // Save to SD card
      if (fsw.LAUNCH) {
        snprintf(fsw.fileName, sizeof(fsw.fileName), "l%07lu.txt", fsw.missionStartTime % 10000000);
      }
      if (fsw.SCIENCE) {
        snprintf(fsw.fileName, sizeof(fsw.fileName), "s%07lu.txt", fsw.missionStartTime % 10000000);
      }
      fsw.currentFile = SD.open(fsw.fileName, FILE_WRITE);
      if (fsw.currentFile && SD.mediaPresent()) {
        fsw.currentFile.print(fsw.fswToSave); // Write FSW data to SD card
        fsw.currentFile.print(fsw.epdsToSave); // Write EPDS data to
        fsw.currentFile.print(fsw.AIToSave); // Write AI data to SD
        fsw.currentFile.print(fsw.histogramAToSave); // Write histogram data to SD card
        fsw.currentFile.print(fsw.histogramBToSave); // Write histogram data to SD card
        fsw.currentFile.println(); // Newline after each set of data
        fsw.currentFile.close();
        Serial.println("[FSW] Data logged to SD card!");
      } else {
        Serial.println("[FSW] ERROR: Could not open file for writing.");
        Serial.println("[FSW] Attempting to deinitialize SD card to recover from error...");
        deinitSDCard(fsw); // Deinitialize SD card to attempt recovery on next log attempt
      }
    }
  }
}