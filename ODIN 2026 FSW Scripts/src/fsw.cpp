#include <TimeLib.h> // RTC LIBRARY
#include <SD.h>
#include <SPI.h>

#include "fsw.h" // FSW HEADER
#include "epds.h" // EPDS HEADER
#include "inst.h" // Spectrometer histogram helpers

// -- INIT FUNCTIONS -- //

// Function to initialize FSW status variables
void initFSWStatus(FSW &fsw) {
  fsw.LAUNCH = true; // TE-2 has not been triggered (Launch Mode)
  fsw.SCIENCE = false; // TE-2 has been triggered (Science Mode)
  fsw.ATTITUDE_RDY = false; // BNO055s have been initialized and are ready to log attitude data
  fsw.RTC_RDY = false; // RTC has been initialized and is ready to provide time data
  fsw.SD_RDY = false; // SD Card has been initialized and is ready to log data
  fsw.HRTBT = false; // Heartbeat LED state (true = on, false = off)
  fsw.TE2_TRIGGERS = 0; // TE2 has been triggered 
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
  setSyncProvider(getTeensy3Time);
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
    DEBUG_SERIAL.println("[FSW] ERROR: Failed to initialize BNO055 A!");
    return false; // FAILED BNO055 A INITIALIZATION
  }
  // Initialize BNO055 B
  fsw.BNO055_B = Adafruit_BNO055(BNO055_SENSOR_ID, BNO055_ADDRESS_B);
  if (!fsw.BNO055_B.begin()) {
    DEBUG_SERIAL.println("[FSW] ERROR: Failed to initialize BNO055 B!");
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
  fsw.lastTransmit = now(); // Last Transmit Time (s)
  fsw.lastPredictionSave = now(); // Last Prediction Saved to SD Card Time (s)

  snprintf(fsw.priorityFileName, sizeof(fsw.priorityFileName),
           "p%07lu.txt", fsw.missionStartTime % 10000000);
}

// -- LOGGING FUNCTIONS -- //

void readAttitude(FSW &fsw) {
  if(!fsw.ATTITUDE_RDY) {
    DEBUG_SERIAL.println("[FSW] ERROR: Cannot log attitude until BNO055 is ready.");
    if (!initBNO055(fsw)) {
      DEBUG_SERIAL.println("[FSW] ERROR: Failed to re-initialize BNO055!");
    } else {
      DEBUG_SERIAL.println("[FSW] Successfully re-initialized BNO055!");
    }
  } else {
    // Read all attitude data from BNO055 A and BNO055 B and store in FSW struct
    fsw.euler_A = fsw.BNO055_A.getVector(Adafruit_BNO055::VECTOR_EULER); // Euler angles from BNO055 A
    fsw.linAcc_A = fsw.BNO055_A.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); // Linear acceleration from BNO055 A
    fsw.mag_A = fsw.BNO055_A.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER); // Magnetometer data from BNO055 A
    fsw.quat_A = fsw.BNO055_A.getQuat(); // Quaternion from BNO055 A

    fsw.euler_B = fsw.BNO055_B.getVector(Adafruit_BNO055::VECTOR_EULER); // Euler angles from BNO055 B
    fsw.linAcc_B = fsw.BNO055_B.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); // Linear acceleration from BNO055 B
    fsw.mag_B = fsw.BNO055_B.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER); // Magnetometer data from BNO055 B
    fsw.quat_B = fsw.BNO055_B.getQuat(); // Quaternion from BNO055 B

    // Print attitude data to Serial Monitor for debugging
    DEBUG_SERIAL.println("[FSW] -- BNO055 A Data --");
    DEBUG_SERIAL.print("[FSW] Euler: ");
    DEBUG_SERIAL.print(fsw.euler_A.x(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(fsw.euler_A.y(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.println(fsw.euler_A.z(), 3);
    DEBUG_SERIAL.print("[FSW] Quaternion: ");
    DEBUG_SERIAL.print(fsw.quat_A.x(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(fsw.quat_A.y(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(fsw.quat_A.z(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.println(fsw.quat_A.w(), 3);
    DEBUG_SERIAL.print("[FSW] Magnetometer: ");
    DEBUG_SERIAL.print(fsw.mag_A.x(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(fsw.mag_A.y(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.println(fsw.mag_A.z(), 3);
    DEBUG_SERIAL.print("[FSW] Linear Acceleration: ");
    DEBUG_SERIAL.print(fsw.linAcc_A.x(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(fsw.linAcc_A.y(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.println(fsw.linAcc_A.z(), 3);

    DEBUG_SERIAL.println("[FSW] -- BNO055 B Data --");
    DEBUG_SERIAL.print("[FSW] Euler: ");
    DEBUG_SERIAL.print(fsw.euler_B.x(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(fsw.euler_B.y(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.println(fsw.euler_B.z(), 3);
    DEBUG_SERIAL.print("[FSW] Quaternion: ");
    DEBUG_SERIAL.print(fsw.quat_B.x(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(fsw.quat_B.y(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(fsw.quat_B.z(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.println(fsw.quat_B.w(), 3);
    DEBUG_SERIAL.print("[FSW] Magnetometer: ");
    DEBUG_SERIAL.print(fsw.mag_B.x(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(fsw.mag_B.y(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.println(fsw.mag_B.z(), 3);
    DEBUG_SERIAL.print("[FSW] Linear Acceleration: ");
    DEBUG_SERIAL.print(fsw.linAcc_B.x(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(fsw.linAcc_B.y(), 3);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.println(fsw.linAcc_B.z(), 3);

    fsw.fswToSave += String(fsw.euler_A.x(), 3) + ";" + String(fsw.euler_A.y(), 3) + ";" + String(fsw.euler_A.z(), 3) + ";" + // Log BNO055 A Euler angles
                      String(fsw.quat_A.x(), 3) + ";" + String(fsw.quat_A.y(), 3) + ";" + String(fsw.quat_A.z(), 3) + ";" + String(fsw.quat_A.w(), 3) + ";" + // Log BNO055 A Quaternion
                      String(fsw.mag_A.x(), 3) + ";" + String(fsw.mag_A.y(), 3) + ";" + String(fsw.mag_A.z(), 3) + ";" + // Log BNO055 A Magnetometer
                      String(fsw.linAcc_A.x(), 3) + ";" + String(fsw.linAcc_A.y(), 3) + ";" + String(fsw.linAcc_A.z(), 3) + ";"; // Log BNO055 A Linear Acceleration

    fsw.fswToSave += String(fsw.euler_B.x(), 3) + ";" + String(fsw.euler_B.y(), 3) + ";" + String(fsw.euler_B.z(), 3) + ";" + // Log BNO055 B Euler angles
                      String(fsw.quat_B.x(), 3) + ";" + String(fsw.quat_B.y(), 3) + ";" + String(fsw.quat_B.z(), 3) + ";" + String(fsw.quat_B.w(), 3) + ";" + // Log BNO055 B Quaternion
                      String(fsw.mag_B.x(), 3) + ";" + String(fsw.mag_B.y(), 3) + ";" + String(fsw.mag_B.z(), 3) + ";" + // Log BNO055 B Magnetometer
                      String(fsw.linAcc_B.x(), 3) + ";" + String(fsw.linAcc_B.y(), 3) + ";" + String(fsw.linAcc_B.z(), 3) ; // Log BNO055 B Linear Acceleration
  }
}

void readEPDS(EPDS &epds, FSW &fsw) {
  if(!epds.initialized) {
    DEBUG_SERIAL.println("[EPDS] ERROR: Cannot log EPDS data until EPDS is initialized.");
    if (EPDS_init(epds)) {
      DEBUG_SERIAL.println("[EPDS] EPDS initialization successful on retry!");
    } else {
      DEBUG_SERIAL.println("[EPDS] EPDS initialization failed again on retry!");
    }
  } else {
    EPDS_readAll(epds);

    // Use voltage values directly, do something with them as needed (e.g., log, check thresholds, etc.)
    float rkt  = epds.RKT_V;
    float v12  = epds.V12_V;
    float v5   = epds.V5_V;
    float v3v3 = epds.V3V3_V;

    // for debugging, print the voltage values to the serial monitor in one line
    DEBUG_SERIAL.print("[EPDS] RKT Voltage: ");
    DEBUG_SERIAL.print(rkt, 3);
    DEBUG_SERIAL.print(" | 12V Bus: ");
    DEBUG_SERIAL.print(v12, 3);
    DEBUG_SERIAL.print(" | 5V Bus: ");
    DEBUG_SERIAL.print(v5, 3);
    DEBUG_SERIAL.print(" | 3V3 Bus: ");
    DEBUG_SERIAL.println(v3v3, 3);

    fsw.epdsToSave += String(rkt, 3) + ";" + String(v12, 3) + ";" + String(v5, 3) + ";" + String(v3v3, 3) ; // Log EPDS voltages
  }
}

void logData(FSW &fsw) {
  // -- Save to SD Card -- //
  if ((fsw.currentMissionTime - fsw.lastSDCardSave) >= SD_SAVE_INTERVAL) {
    if(!fsw.SD_RDY) {
      DEBUG_SERIAL.println("[FSW] ERROR: Cannot log data until SD card is ready.");
      if(!initSDCard(fsw)) {
        DEBUG_SERIAL.println("[FSW] ERROR: Attempted SD card initialization and failed!");
      } else {
        DEBUG_SERIAL.println("[FSW] SD card initialization successful!");
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
        DEBUG_SERIAL.println("[FSW] Data logged to SD card!");
      } else {
        DEBUG_SERIAL.println("[FSW] ERROR: Could not open file for writing.");
        DEBUG_SERIAL.println("[FSW] Attempting to deinitialize SD card to recover from error...");
        deinitSDCard(fsw); // Deinitialize SD card to attempt recovery on next log attempt
      }
    }
  }
}

// -- PRIORITY QUEUE (ORIN predictions) -- //

static void priorityStripCR(String &s) {
  while (s.length() > 0) {
    char c = s[s.length() - 1];
    if (c != '\r' && c != '\n') break;
    s.remove(s.length() - 1);
  }
}

// Copy src -> dst byte-for-byte, then remove src. Used in place of SD.rename
// because the stock Arduino SD wrapper does not always expose it.
static bool priorityReplaceFile(const char *src, const char *dst) {
  if (!SD.exists(src)) return false;
  if (SD.exists(dst)) SD.remove(dst);
  File in  = SD.open(src, FILE_READ);
  File out = SD.open(dst, FILE_WRITE);
  if (!in || !out) {
    if (in) in.close();
    if (out) out.close();
    return false;
  }
  uint8_t buf[256];
  while (in.available()) {
    int n = in.read(buf, sizeof(buf));
    if (n <= 0) break;
    out.write(buf, n);
  }
  in.close();
  out.close();
  SD.remove(src);
  return true;
}

float Priority_ParseTopConfidence(const String &orinLine) {
  int comma = orinLine.indexOf(',');
  if (comma < 0) return 0.0f;
  int semi = orinLine.indexOf(';', comma + 1);
  String token = (semi < 0)
      ? orinLine.substring(comma + 1)
      : orinLine.substring(comma + 1, semi);
  token.trim();
  return token.toFloat();
}

static void priorityWriteRow(File &out, const String &orinLine, const String &hist) {
  out.print(orinLine);
  out.print('|');
  out.print(hist);
  out.print('\n');
}

bool Priority_Insert(FSW &fsw, const String &orinLine, const String &combinedHist) {
  if (!fsw.SD_RDY) return false;

  float newConf = Priority_ParseTopConfidence(orinLine);

  // First insert: file doesn't exist yet, write the new row as the only line.
  if (!SD.exists(fsw.priorityFileName)) {
    File out = SD.open(fsw.priorityFileName, FILE_WRITE);
    if (!out) return false;
    priorityWriteRow(out, orinLine, combinedHist);
    out.close();
    return true;
  }

  // Stream-merge existing file + new row into PRIORITY_TMP_FILE, sorted desc.
  SD.remove(PRIORITY_TMP_FILE);
  File in  = SD.open(fsw.priorityFileName, FILE_READ);
  File out = SD.open(PRIORITY_TMP_FILE, FILE_WRITE);
  if (!in || !out) {
    if (in) in.close();
    if (out) out.close();
    return false;
  }

  int  rowsWritten = 0;
  bool inserted    = false;

  while (in.available() && rowsWritten < PRIORITY_MAX_ROWS) {
    String line = in.readStringUntil('\n');
    priorityStripCR(line);
    if (line.length() == 0) continue;

    float existingConf = Priority_ParseTopConfidence(line);

    if (!inserted && newConf >= existingConf) {
      priorityWriteRow(out, orinLine, combinedHist);
      rowsWritten++;
      inserted = true;
      if (rowsWritten >= PRIORITY_MAX_ROWS) break;
    }

    out.print(line);
    out.print('\n');
    rowsWritten++;
  }

  // New row was the lowest-confidence and there's still room: append at end.
  if (!inserted && rowsWritten < PRIORITY_MAX_ROWS) {
    priorityWriteRow(out, orinLine, combinedHist);
    rowsWritten++;
  }

  in.close();
  out.close();

  SD.remove(fsw.priorityFileName);
  return priorityReplaceFile(PRIORITY_TMP_FILE, fsw.priorityFileName);
}

bool Priority_PeekTop(FSW &fsw, String &orinLineOut, String &histOut) {
  if (!fsw.SD_RDY) return false;
  if (!SD.exists(fsw.priorityFileName)) return false;
  File f = SD.open(fsw.priorityFileName, FILE_READ);
  if (!f) return false;
  if (!f.available()) { f.close(); return false; }

  String line = f.readStringUntil('\n');
  f.close();
  priorityStripCR(line);
  if (line.length() == 0) return false;

  int sep = line.indexOf('|');
  if (sep < 0) return false;
  orinLineOut = line.substring(0, sep);
  histOut     = line.substring(sep + 1);
  return true;
}

bool Priority_DeleteTop(FSW &fsw) {
  if (!fsw.SD_RDY) return false;
  if (!SD.exists(fsw.priorityFileName)) return false;

  SD.remove(PRIORITY_TMP_FILE);
  File in  = SD.open(fsw.priorityFileName, FILE_READ);
  File out = SD.open(PRIORITY_TMP_FILE, FILE_WRITE);
  if (!in || !out) {
    if (in) in.close();
    if (out) out.close();
    return false;
  }

  bool first = true;
  bool wroteAnything = false;
  while (in.available()) {
    String line = in.readStringUntil('\n');
    priorityStripCR(line);
    if (first) { first = false; continue; }   // drop the top row
    if (line.length() == 0) continue;
    out.print(line);
    out.print('\n');
    wroteAnything = true;
  }

  in.close();
  out.close();

  SD.remove(fsw.priorityFileName);

  if (!wroteAnything) {
    SD.remove(PRIORITY_TMP_FILE);
    return true; // queue is now empty
  }
  return priorityReplaceFile(PRIORITY_TMP_FILE, fsw.priorityFileName);
}