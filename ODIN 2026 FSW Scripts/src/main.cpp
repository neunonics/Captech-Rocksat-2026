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

// Globals
EPDS epds; // EPDS struct instance
FSW fsw; // FSW struct instance
COMM comm; // COMMS struct instance
Histogram     hist1, hist2; // Define histogram instances
CombinedHistogram combined; // Define combined histogram instance

// Functions
void te2(); // Interrupt function that runs when TE-2 is triggered

void setup() {
  initPins(); // Setup Pins
  digitalWrite(LED_PWR, HIGH); // Turn on power led
  fsw.lastHeartbeatTime = now(); // Initialize last heartbeat time

  Serial.begin(115200); //Starts the serial monitor for debugging
  Wire.begin(); //tells the computer to start the I2C
  
  //FSW Startup Message
  Serial.println("ODIN FSW Starting Up...");

  // Initialize Struct Values
  initFSWStatus(fsw); // Initialize FSW status variables (Launch, Science, Attitude Ready, RTC Ready)
  initCOMMStatus(comm); // Initialize COMMS status variables (ENBL_STATUS, TX_ACTIVE, messagesSent)

  // -- Initialize SD Card -- //
  while (!SD.begin()) { ; } // Wait for serial port to connect

  Serial.println("Initializing SD card...");
  for (int attempt = 0; attempt < 3 && !fsw.SD_RDY; attempt++) {
    initSDCard(fsw);
  }
  if (!fsw.SD_RDY) {
    Serial.println("SD card initialization failed after 3 attempts!");
  } else {
    Serial.println("SD card initialization done.");

  }

  // -- Initialize RTC -- //
  Serial.println("Initializing RTC...");
  for (int attempt = 0; attempt < 3 && !fsw.RTC_RDY; attempt++) {
    initRTC(fsw);
  }
  if (!fsw.RTC_RDY) {
    Serial.println("RTC initialization failed after 3 attempts!");
  } else {
    Serial.println("RTC initialization done.");
  }
  Serial.print("Mission Start Time: ");
  Serial.println(fsw.missionStartTime);  

  // -- Initialize BNO055 A & B-- //
  Serial.println("Initializing BNO055 A & B...");
  for (int attempt = 0; attempt < 3 && !fsw.ATTITUDE_RDY; attempt++) {
    initBNO055(fsw);
  }
  if (!fsw.ATTITUDE_RDY) {
    Serial.println("BNO055 initialization failed after 3 attempts!");
  } else {
    Serial.println("BNO055 initialization done.");
  }

  // --Initialize Spectrometers A & B -- //
  Serial.println("Initializing Spectrometers A & B...");

  SPEC_InitUntilConnected();
  SPEC_SyncReboot();


  // -- Initialize EPDS -- //
  Serial.println("Initializing EPDS...");
  for (int attempt = 0; attempt < 3 && !epds.initialized; attempt++) {
        EPDS_init(epds);
  }
  if (!epds.initialized) {
        Serial.println("EPDS initialization failed after 3 attempts!");
  } else {
        Serial.println("EPDS initialization done.");
  }
  
  // -- Initialize COMMS -- //
  Serial.println("Initializing COMMS...");
  attachInterrupt(TE2_SIGNAL, te2, CHANGE); // This takes the TE2 signal, telling the computer to watch when to begin the TE2 program (this TE2 program is a function)
  if(digitalRead(COMM_BTD) == HIGH) {
    Serial.println("COMMS may have been enabled in prior run. Powering down COMMS to reset...");
    commShutDown(comm); // Not guaranteed, but likely to work!
  }

  // Try to initialize COMMS up to 3 times, in case of failure
  for (int attempt = 0; attempt < 3 && !comm.ENBL_STATUS; attempt++) {
    initCOMM(comm);
  }
  if (!comm.ENBL_STATUS) {
    Serial.println("COMMS initialization failed after 3 attempts!");
  } else {
    Serial.println("COMMS initialization done.");
  }
}

void loop() {
  if(!fsw.RTC_RDY) {
    Serial.println("ERROR: Cannot log time until RTC is ready.");
  } else {
    // -- Log Time -- //
    fsw.currentMissionTime = now(); // Get the current time
    Serial.print("Current Mission Time: ");
    Serial.println(fsw.currentMissionTime);
    fsw.lineToSave += String(fsw.currentMissionTime) + ";"; // Log time as delta from mission start
  }

  if (fsw.currentMissionTime - fsw.lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    fsw.lastHeartbeatTime = fsw.currentMissionTime; // Update last heartbeat time
    digitalWrite(LED_HRTBT, !digitalRead(LED_HRTBT)); // Heartbeat LED On
  }
  digitalWrite(LED_HRTBT, HIGH); // Heartbeat LED On
  digitalWrite(LED_HRTBT, LOW); // THeartbeat LED Off
  
  fsw.lineToSave = ""; // Reset Line to Save to SD Card
  
  if(fsw.LAUNCH){
    // ODIN has been powered & no TE-2 Signal (LAUNCH Mode)

    if(!fsw.ATTITUDE_RDY) {
      Serial.println("ERROR: Cannot log attitude until BNO055 is ready.");
    } else {
      // -- Log Attitude -- //
    }

    if(!epds.initialized) {
        Serial.println("ERROR: Cannot log EPDS data until EPDS is initialized.");
    } else {
      EPDS_readAll(epds);
 
      // Use voltage values directly, do something with them as needed (e.g., log, check thresholds, etc.)
      float rkt  = epds.RKT_V;
      float v12  = epds.V12_V;
      float v5   = epds.V5_V;
      float v3v3 = epds.V3V3_V;

      // for debugging, print the voltage values to the serial monitor in one 
      Serial.print("RKT Voltage: ");
      Serial.print(rkt, 3);
      Serial.print(" | 12V Bus: ");
      Serial.print(v12, 3);
      Serial.print(" | 5V Bus: ");
      Serial.print(v5, 3);
      Serial.print(" | 3V3 Bus: ");
      Serial.println(v3v3, 3);

      fsw.lineToSave += String(rkt, 3) + ";" + String(v12, 3) + ";" + String(v5, 3) + ";" + String(v3v3, 3) + ";"; // Log EPDS voltages
      
    }
    

    // -- Save to SD Card -- //
    if(!fsw.SD_RDY) {
      Serial.println("ERROR: Cannot log data until SD card is ready.");
    } else {
      // Save lineToSave to SD card
      fsw.launchFile = SD.open("datalog.txt", FILE_WRITE);
      if (fsw.launchFile) {
        fsw.launchFile.println(fsw.lineToSave);
        fsw.launchFile.close();
        Serial.println("Data logged to SD card: " + fsw.lineToSave);
      } else {
        Serial.println("ERROR: Could not open datalog.txt for writing.");
      }
    }
  }
  if(fsw.SCIENCE){
    // TE-2 has been triggered (Science Mode)
    SPEC_ReadHistogram1(hist1);
    SPEC_ReadHistogram2(hist2);

    SPEC_CombineHistograms(hist1, hist2, combined);

    SPEC_ScrubNaN(combined);

    SPEC_TransmitCombined(combined);

    SPEC_CheckQuality(hist1);
    SPEC_CheckQuality(hist2);
    SPEC_HandleFaults(hist1, hist2);

    if(!fsw.SD_RDY) {
      Serial.println("ERROR: Cannot log data until SD card is ready.");
    } else {
      // Save lineToSave to SD card
      fsw.scienceFile = SD.open("scienceFile.txt", FILE_WRITE);
      if (fsw.scienceFile) {
        fsw.scienceFile.println(fsw.lineToSave);
        fsw.scienceFile.close();
        Serial.println("Data logged to SD card: " + fsw.lineToSave);
      } else {
        Serial.println("ERROR: Could not open scienceFile.txt for writing.");
      }
    }
  }

}

void te2(){
  // Interrupt function that runs when TE-2 is triggered
  delay(TE2_WAIT); // Wait at least 10 seconds before
  fsw.LAUNCH = false; // TE-2 has been triggered (Science Mode)
  fsw.SCIENCE = true; // TE-2 has been triggered (Science Mode)
  digitalWrite(LED_COMM, HIGH); // Turn on COMMS LED to indicate TE-2 has been triggered and COMMS is active
  SPEC_SyncReboot();
}