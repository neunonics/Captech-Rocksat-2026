// LIBRARIES

#include <Arduino.h> // STANDARD ARDUINO LIBRARY
#include <Wire.h> // WIRE LIBRARY

#include <SD.h> // SD CARD
#include <SPI.h> // SD CARD DATA PROTOCOL

#include <TimeLib.h> // RTC LIBRARY

#include <Adafruit_Sensor.h> // BNO055 LIBRARY
#include <Adafruit_BNO055.h> // BNO055 LIBRARY
#include <utility/imumaths.h> // BNO055 LIBRARY

#include <rockblock_9704.h> // IRIDIUM 9704 LIBRARY

#include "pins.h" // FSW PINS HEADER
#include "epds.h" // EPDS HEADER
#include "fsw.h" // FSW HEADER
#include "comm.h" // COMMS HEADER
#include "inst.h" // INSTRUMENTS HEADER 

// CONSTANTS & DEFINES
#define TE2_WAIT 10000    // Wait at least 10 seconds before arming TE-2, just to make sure the payload isn't triggered early when starting
#define TEST_WAIT 1000    // Wait 1 second for testing

// Globals
EPDS epds; // EPDS struct instance
FSW fsw; // FSW struct instance
COMM comm; // COMMS struct instance
extern Histogram     hist1, hist2; // Define histogram instances
extern CombinedHistogram combined; // Define combined histogram instance

// Functions
void te2(); // Interrupt function that runs when TE-2 is triggered

void setup() {
  initPins(); // Setup Pins
  digitalWrite(LED_PWR, HIGH); // Turn on power led

  Serial.begin(9600); //Starts the serial monitor for debugging
  OUTPUT_SERIAL.begin(OUTPUT_BAUD); // Serial for transmitting combined histogram data

  while(!Serial) { // Wait for serial port to connect
    delay(10);
  }

  Serial.println("Serial Started...");
  Wire.begin(); //tells the computer to start the I2C
  
  //FSW Startup Message
  Serial.println("ODIN FSW Starting Up...");

  // Initialize Struct Values
  initFSWStatus(fsw); // Initialize FSW status variables (Launch, Science, Attitude Ready, RTC Ready)
  initCOMMStatus(comm); // Initialize COMMS status variables (ENBL_STATUS, TX_ACTIVE, messagesSent)

  // -- Initialize RTC -- //
  while (!fsw.RTC_RDY) {
     Serial.println("[FSW] RTC not ready, attempting initialization...");
     if (!initRTC(fsw)) {
       Serial.println("[FSW] RTC initialization attempt failed!");
     } else {
       Serial.println("[FSW] RTC initialization successful!");
     }
     delay(10); // Short delay before retrying RTC initialization
  }
  initTimers(fsw); // Initialize mission timers
  Serial.print("[FSW] Mission Start Time: ");
  Serial.println(fsw.missionStartTime);  
  Serial.print("[FSW] Current Mission Time: ");
  Serial.println(fsw.currentMissionTime);
  Serial.print("[FSW] Last Heartbeat Time: ");
  Serial.println(fsw.lastHeartbeatTime);
  Serial.print("[FSW] Last SD Card Save Time: ");
  Serial.println(fsw.lastSDCardSave);

  // -- Initialize SD Card -- //

  Serial.println("[FSW] Initializing SD card...");
  for (int attempt = 0; attempt < 3 && !fsw.SD_RDY; attempt++) {
    initSDCard(fsw);
  }
  if (!fsw.SD_RDY) {
    Serial.println("[FSW] SD card initialization failed after 3 attempts!");
  } else {
    Serial.println("[FSW] SD card initialization done.");
  }

  // -- Initialize BNO055 A & B-- //
  Serial.println("[FSW] Initializing BNO055 A & B...");
  for (int attempt = 0; attempt < 3 && !fsw.ATTITUDE_RDY; attempt++) {
    initBNO055(fsw);
  }
  if (!fsw.ATTITUDE_RDY) {
    Serial.println("[FSW] BNO055 initialization failed after 3 attempts!");
  } else {
    Serial.println("[FSW] BNO055 initialization done.");
  }

  // --Initialize Spectrometers A & B -- //
  Serial.println("[SPEC] Initializing Spectrometers A & B...");
  
  SPEC_InitUntilConnected();
  SPEC_SyncReboot();

  // -- Initialize EPDS -- //
  Serial.println("[EPDS] Initializing EPDS...");
  for (int attempt = 0; attempt < 3 && !epds.initialized; attempt++) {
        EPDS_init(epds);
  }
  if (!epds.initialized) {
        Serial.println("[EPDS] EPDS initialization failed after 3 attempts!");
  } else {
        Serial.println("[EPDS] EPDS initialization done.");
  }
  
  // -- Initialize COMMS -- //
  Serial.println("[COMM] Initializing COMMS...");

  // Try to initialize COMMS up to 3 times, in case of failure
  for (int attempt = 0; attempt < 3 && !comm.ENBL_STATUS; attempt++) {
    //initCOMM(comm);
  }
  if (!comm.ENBL_STATUS) {
    Serial.println("[COMM] COMMS initialization failed after 3 attempts!");
  } else {
    Serial.println("[COMM] COMMS initialization done.");
  }

  delay(10000);

  attachInterrupt(TE2_SIGNAL, te2, RISING); // This takes the TE2 signal, telling the computer to watch when to begin the TE2 program (this TE2 program is a function)
}

void loop() {
  delay(TEST_WAIT); // Short delay for testing, adjust as needed for actual mission timing

  fsw.fswToSave = ""; // Reset FSW data to save
  fsw.epdsToSave = ""; // Reset EPDS data to save
  fsw.histogramAToSave = ""; // Reset histogram data to save
  fsw.histogramBToSave = ""; // Reset histogram data to save
  fsw.AIToSave = ""; // Reset AI data to save

  // -- Log Time -- //
  fsw.currentMissionTime = now(); // Get the current time
  Serial.print("[FSW] Current Mission Time: ");
  Serial.println(fsw.currentMissionTime);
  fsw.fswToSave += String(fsw.currentMissionTime) + ";"; // Log time as delta from mission start

  if ((fsw.currentMissionTime - fsw.lastHeartbeatTime) >= HEARTBEAT_INTERVAL) {
    fsw.lastHeartbeatTime = now(); // Update last heartbeat time
    pinMode(LED_HRTBT, INPUT); // Set heartbeat LED pin as output
    if (digitalRead(LED_HRTBT) == LOW) {
      pinMode(LED_HRTBT, OUTPUT); // Set heartbeat LED pin as output
      digitalWrite(LED_HRTBT, HIGH); // Heartbeat LED Off
    } else {
      pinMode(LED_HRTBT, OUTPUT); // Set heartbeat LED pin as output
      digitalWrite(LED_HRTBT, LOW); // Heartbeat LED On
    }
    digitalWrite(LED_HRTBT, !digitalRead(LED_HRTBT)); // Heartbeat LED On
  }
  
  readAttitude(fsw); // Checks if BNO055s are ready and reads attitude data if they are, otherwise logs error message
  readEPDS(epds, fsw); // Checks if EPDS is ready and reads EPDS data if it is, otherwise logs error message

  if(fsw.LAUNCH){
    // ODIN has been powered & no TE-2 Signal (LAUNCH Mode)
  }

  if(fsw.SCIENCE){
    // TE-2 has been triggered (Science Mode)

    // -- See if ORIN has new prediction -- //
    ORIN_Poll();

    // -- Read Spectrometer Histograms -- //
    Serial.println("[SPEC] Reading spectrometer histograms...");
    SPEC_ReadHistogram1(hist1);
    SPEC_ReadHistogram2(hist2);
    Serial.println("[SPEC] Combining histograms...");
    SPEC_CombineHistograms(hist1, hist2, combined);
    Serial.println("[SPEC] Scrubbing NaNs from combined histogram...");
    SPEC_ScrubNaN(combined);
    Serial.println("[SPEC] Transmitting combined histogram...");
    SPEC_TransmitCombined(combined);
    Serial.println("[SPEC] Checking histogram quality and handling faults...");
    SPEC_CheckQuality(hist1);
    SPEC_CheckQuality(hist2);
    SPEC_HandleFaults(hist1, hist2);

    for (int bin = 0; bin < HISTOGRAM_BINS; bin++) {
        fsw.histogramAToSave += String(hist1.bins[bin]) + ";";
        fsw.histogramBToSave += String(hist2.bins[bin]) + ";";
    }
  }

  // -- Log Data to SD Card -- //
  logData(fsw); // !! END OF MAIN LOOP !!
}

void te2(){
  if (fsw.LAUNCH) {
    // Interrupt function that runs when TE-2 is triggered
    Serial.println("[FSW] TE-2 Signal Read! Payload now transitioning to Science Mode...");
    delay(TE2_WAIT); // Wait at least 10 seconds before
    fsw.LAUNCH = false; // TE-2 has been triggered (Science Mode)
    fsw.SCIENCE = true; // TE-2 has been triggered (Science Mode)
    digitalWrite(LED_COMM, HIGH); // Turn on COMMS LED to indicate TE-2 has been triggered and COMMS is now primed to XMIT
    SPEC_SyncReboot();
    fsw.fswToSave = ""; // Reset FSW data to save
    fsw.epdsToSave = ""; // Reset EPDS data to save
    fsw.histogramAToSave = ""; // Reset histogram data to save
    fsw.histogramBToSave = ""; // Reset histogram data to save
    fsw.AIToSave = ""; // Reset AI data to save
    Serial.println("[FSW] TE-2 Setup Finished! Payload now in Science Mode...");
  } else {
    return; // If TE-2 is triggered again, do nothing (already in Science Mode)
  }
}