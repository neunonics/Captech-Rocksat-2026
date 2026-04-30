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
#define SETUP_WAIT 1000 // Wait 1 second between setup steps
#define LOOP_WAIT 10 // Wait 10 milliseconds between loop iterations
#define TE2_WAIT 5000    // Wait at least 5 seconds before arming TE-2, just to make sure the payload isn't triggered early when starting
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
  // -- Standard Wiring Initialization -- //
  DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUD); // Starts the serial monitor for debugging
  COMM_SERIAL.begin(COMM_BAUD_RATE); // Serial for transmitting through COMM
  SPEC1_SERIAL.begin(SPEC_BAUD); // Serial for Spectrometer 1
  SPEC2_SERIAL.begin(SPEC_BAUD); // Serial for Spectrometer 2
  OUTPUT_SERIAL.begin(OUTPUT_BAUD); // Serial for transmitting combined histogram data

  initPins(); // Setup Pins
  digitalWrite(LED_PWR, HIGH); // Turn on power led

  delay(SETUP_WAIT); // Short delay to ensure serial is ready

  DEBUG_SERIAL.println("Serial Started...");
  Wire.begin(); //tells the computer to start the I2C
  
  //FSW Startup Message
  DEBUG_SERIAL.println("ODIN FSW Starting Up...");

  // Initialize Struct Values
  initFSWStatus(fsw); // Initialize FSW status variables (Launch, Science, Attitude Ready, RTC Ready)
  initCOMMStatus(comm); // Initialize COMMS status variables (ENBL_STATUS, TX_ACTIVE, messagesSent)

  // -- Initialize RTC -- //
  while (!fsw.RTC_RDY) {
     DEBUG_SERIAL.println("[FSW] RTC not ready, attempting initialization...");
     if (!initRTC(fsw)) {
       DEBUG_SERIAL.println("[FSW] RTC initialization attempt failed!");
     } else {
       DEBUG_SERIAL.println("[FSW] RTC initialization successful!");
     }
     delay(LOOP_WAIT); // Short delay before retrying RTC initialization
  }
  initTimers(fsw); // Initialize mission timers
  DEBUG_SERIAL.print("[FSW] Mission Start Time: ");
  DEBUG_SERIAL.println(fsw.missionStartTime);

  // -- Initialize SD Card -- //

  DEBUG_SERIAL.println("[FSW] Initializing SD card...");
  for (int attempt = 0; attempt < 3 && !fsw.SD_RDY; attempt++) {
    initSDCard(fsw);
    delay(LOOP_WAIT);
  }
  if (!fsw.SD_RDY) {
    DEBUG_SERIAL.println("[FSW] SD card initialization failed after 3 attempts!");
  } else {
    DEBUG_SERIAL.println("[FSW] SD card initialization done.");
  }

  // -- Initialize BNO055 A & B-- //
  DEBUG_SERIAL.println("[FSW] Initializing BNO055 A & B...");
  for (int attempt = 0; attempt < 3 && !fsw.ATTITUDE_RDY; attempt++) {
    initBNO055(fsw);
    delay(LOOP_WAIT);
  }
  if (!fsw.ATTITUDE_RDY) {
    DEBUG_SERIAL.println("[FSW] BNO055 initialization failed after 3 attempts!");
  } else {
    DEBUG_SERIAL.println("[FSW] BNO055 initialization done.");
  }

  // --Initialize Spectrometers A & B -- //
  DEBUG_SERIAL.println("[SPEC] Initializing Spectrometers A & B...");
  
  SPEC_InitUntilConnected();
  SPEC_SyncReboot();

  // -- Initialize EPDS -- //
  DEBUG_SERIAL.println("[EPDS] Initializing EPDS...");
  for (int attempt = 0; attempt < 3 && !epds.initialized; attempt++) {
        EPDS_init(epds);
        delay(LOOP_WAIT);
  }
  if (!epds.initialized) {
        DEBUG_SERIAL.println("[EPDS] EPDS initialization failed after 3 attempts!");
  } else {
        DEBUG_SERIAL.println("[EPDS] EPDS initialization done.");
  }
  
  // -- Initialize COMMS -- //
  DEBUG_SERIAL.println("[COMM] Initializing COMMS...");

  // Try to initialize COMMS up to 3 times, in case of failure
  for (int attempt = 0; attempt < 3 && !comm.ENBL_STATUS; attempt++) {
    initCOMM(comm);
    delay(LOOP_WAIT);
  }
  if (!comm.ENBL_STATUS) {
    DEBUG_SERIAL.println("[COMM] COMMS initialization failed after 3 attempts!");
  } else {
    DEBUG_SERIAL.println("[COMM] COMMS initialization done.");
  }
}

void loop() {
  delay(TEST_WAIT); // Short delay for testing !! REMOVE BEFORE FLIGHT !!

  if(fsw.LAUNCH){
    // ODIN has been powered & no TE-2 Signal (LAUNCH Mode)
    if ((fsw.currentMissionTime - fsw.lastTE2CheckTime) >= TE2_CHECK_INTERVAL) {
      fsw.lastTE2CheckTime = now(); // Update last TE-2 check time
      if (digitalRead(TE2_SIGNAL) == HIGH) {
        fsw.TE2_TRIGGERS++; // Increment TE-2 trigger count
        DEBUG_SERIAL.println("[FSW] TE-2 Signal Read! Incrementing TE-2 trigger count. Current TE-2 trigger count: " + String(fsw.TE2_TRIGGERS));
      }
    }

    if (fsw.TE2_TRIGGERS >= TE_2_TRIGGER_THRESHOLD) { // If TE-2 signal has been read high at least threshold times, transition to science mode
      te2(); // TE-2 has been triggered (Science Mode)
    }
  }

  fsw.fswToSave = ""; // Reset FSW data to save
  fsw.epdsToSave = ""; // Reset EPDS data to save
  fsw.histogramAToSave = ""; // Reset histogram data to save
  fsw.histogramBToSave = ""; // Reset histogram data to save
  fsw.AIToSave = ""; // Reset AI data to save
  fsw.combinedHistogram = ""; // Reset Combined Histogram to Transmit

  // -- Log Time -- //
  fsw.currentMissionTime = now(); // Get the current time
  DEBUG_SERIAL.print("[FSW] Current Mission Time: ");
  DEBUG_SERIAL.println(fsw.currentMissionTime);
  fsw.fswToSave += String(fsw.currentMissionTime) + ";";

  if ((fsw.currentMissionTime - fsw.lastHeartbeatTime) >= HEARTBEAT_INTERVAL) {
    fsw.lastHeartbeatTime = now(); // Update last heartbeat time

    if(fsw.HRTBT) {
      digitalWrite(LED_HRTBT, LOW); // Heartbeat LED On
    } else {
      digitalWrite(LED_HRTBT, HIGH); // Heartbeat LED Off
    }
    fsw.HRTBT = !fsw.HRTBT; // Flip heartbeat state for next toggle
  }
  
  readAttitude(fsw); // Checks if BNO055s are ready and reads attitude data if they are, otherwise logs error message
  readEPDS(epds, fsw); // Checks if EPDS is ready and reads EPDS data if it is, otherwise logs error message

  if(fsw.SCIENCE){
    // TE-2 has been triggered (Science Mode)

    // -- See if ORIN has new prediction -- //
    fsw.AIToSave = ORIN_Poll();

    // -- Read Spectrometer Histograms -- //
    DEBUG_SERIAL.println("[SPEC] Reading spectrometer histograms...");
    SPEC_ReadHistogram1(hist1);
    SPEC_ReadHistogram2(hist2);
    DEBUG_SERIAL.println("[SPEC] Combining histograms...");
    SPEC_CombineHistograms(hist1, hist2, combined);
    DEBUG_SERIAL.println("[SPEC] Scrubbing NaNs from combined histogram...");
    SPEC_ScrubNaN(combined);
    DEBUG_SERIAL.println("[SPEC] Transmitting combined histogram...");
    SPEC_TransmitCombined(combined);
    DEBUG_SERIAL.println("[SPEC] Checking histogram quality and handling faults...");
    SPEC_CheckQuality(hist1);
    SPEC_CheckQuality(hist2);
    SPEC_HandleFaults(hist1, hist2);

    for (int bin = 0; bin < HISTOGRAM_BINS; bin++) {
        fsw.histogramAToSave += String(hist1.bins[bin]) + ";";
        fsw.histogramBToSave += String(hist2.bins[bin]) + ";";
        fsw.combinedHistogram += String(combined.bins[bin]) + ";";
    }
    fsw.AIToSave += fsw.combinedHistogram;

    if(comm.messagesSent == 0) {
      sendMessage(fsw, comm);
    }
  }

  // -- Log Data to SD Card -- //
  logData(fsw); // !! END OF MAIN LOOP !!
}

void te2(){
  // TE-2 has been triggered (Enter Science Mode)
  DEBUG_SERIAL.println("[FSW] TE-2 Signal Read! Payload now transitioning to Science Mode...");
  digitalWrite(LED_PWR, LOW); // Turn off power LED to indicate TE-2 has been triggered and payload is now in Science Mode

  fsw.LAUNCH = false; // TE-2 has been triggered (Science Mode)
  fsw.SCIENCE = true; // TE-2 has been triggered (Science Mode)

  digitalWrite(LED_COMM, HIGH); // Turn on COMMS LED to indicate TE-2 has been triggered and COMMS is now primed to XMIT
  SPEC_SyncReboot(); // Reboot spectrometers to ensure they are in sync and ready for science operations
  DEBUG_SERIAL.println("[FSW] TE-2 Setup Finished! Payload now in Science Mode...");
  delay(TE2_WAIT); // Wait at least 10 seconds before starting TE-2 operations
  digitalWrite(LED_PWR, HIGH); // Turn off COMMS LED to indicate TE-2 operations are now starting
}