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

#include "pins.h"

// -- EPDS STUFF -- //
#include "EPDS.h" // EPDS HEADER
EPDS epds;

// -- FSW STUFF -- //
#include "fsw.h" // FSW HEADER
FSW fsw;

// CONSTANTS & DEFINES
#define TE2_WAIT 10000    // Wait at least 10 seconds before arming TE-2, just to make sure the payload isn't triggered early when starting

//Globals (Variables)

File launchFile; // Launch File for SD Card
File scienceFile; // Science File for SD Card

// Functions
void te2(void); //
void iridiumStartup(); //Starts up the Iridium 9704 modem
void iridiumShutdown(); //Shuts down the Iridium 9704 modem


void setup() {
  Serial.begin(115200); //Starts the serial monitor for debugging
  Wire.begin(); //tells the computer to start the I2C
  
  //FSW Startup Message
  Serial.println("ODIN FSW Starting Up...");

  // Initialize FSW struct members
  initFSWStatus(fsw); // Initialize FSW status variables (Launch, Science, Attitude Ready, RTC Ready)

  // -- Initialize SD Card -- //
  while (!Serial) { ; } // Wait for serial port to connect

  Serial.print("Initializing SD card...");

  // Use BUILTIN_SDCARD for the on-board slot
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("Initialization done.");

  // -- Initialize RTC -- //
  Serial.println("Initializing RTC...");
  for (int attempt = 0; attempt < 3 && !fsw.RTC_RDY; attempt++) {
    initTime(fsw);
  }
  if (!fsw.RTC_RDY) {
    Serial.println("RTC initialization failed after 3 attempts!");
  } else {
    Serial.println("RTC initialization done.");
  }
  Serial.print("Mission Start Time: ");
  Serial.println(String(fsw.missionStartTime));  

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
  attachInterrupt(TE2_ISOLATED, te2, CHANGE); // This takes the TE2 signal, telling the computer to watch when to begin the TE2 program (this TE2 program is a function)
}

void loop() {
  // put your main code here, to run repeatedly:
  //Check to see if TE-2 is pulled high
  

  String lineToSave; // String to carry next line to save to the SD Card
  if(fsw.LAUNCH){
    // ODIN has been powered & no TE-2 Signal (LAUNCH Mode)
    fsw.lineToSave = ""; // Clear lineToSave for next SD card line

    // -- Log Time -- //
    fsw.currentMissionTime = now(); // Get the current time
    unsigned long currentTime = now(); // Get the current time in seconds since mission start
    fsw.lineToSave = currentTime;

    // -- Log Attitude -- //
    fsw.lineToSave = fsw.lineToSave + " | " + logAttitude(); // Logs both BNO055s' attitude data to lineToSave

    // -- Log EPDS -- //
    if (!epds.initialized) return;
 
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

    // -- Save to SD Card -- //

  }
  if(fsw.SCIENCE){
    // TE-2 has been triggered (Science Mode)

  }

}

void te2(FSW &fsw){
  // Interrupt function that runs when TE-2 is triggered
  delay(TE2_WAIT); // Wait at least 10 seconds before
  fsw.LAUNCH = false; // TE-2 has been triggered (Science Mode)
  fsw.SCIENCE = true; // TE-2 has been triggered (Science Mode

  //original dan work
  //sys.status.enable_iridium = true;
  //sys.status.enable_spectrometer = true;
  //digitalWrite(ENA_SPECTRO, HIGH);
  //digitalWrite(ENA_IRIDIUM, HIGH);
}