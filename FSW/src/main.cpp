#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <STM32RTC.h>
#include "rockblock_9704.h"
#include "pins.h"
#include "data.h"
#include <map>
#include<string>

//DEFINES
#define MIN_TE2_WAIT 10    // Wait at least 10 seconds before arming TE-2, just to make sure the payload isn't triggered early when starting


//Globals
JsonDocument doc; //Variable
FSW_SYSTEM sys; //Variable
STM32RTC& rtc = STM32RTC::getInstance(); //Variable


//Prototypes
void te2(void);


void setup() {
  //Set Mission Start time
  sys.mission_start.setFromRTC(rtc);

  //Setup Pins //Separate LEDs for the system
  pinMode(TE2_ISOLATED, INPUT); //the input is coming from the computer inputting the signal to the Iridium and the spectrometer
  pinMode(ENA_IRIDIUM, OUTPUT); // Pins h and main update
  pinMode(ENA_SPECTRO, OUTPUT); // This shows that the spectrometer is on (through LED blinks)
  pinMode(LED_POWER, OUTPUT); // LED is powered on
  pinMode(LED_HRTBT, OUTPUT);  // Blink once in a while indicating that the system is on
  pinMode(LED_SDACTIVE, OUTPUT); // LED turns on indicating that the SD card is actively reading data
  pinMode(LED_ERROR, OUTPUT); // LED turns on indicating an error
  pinMode(LED_COMM, OUTPUT); // LED turns on showing that the antenna placed an output

  digitalWrite(ENA_IRIDIUM, LOW);//turns off
  digitalWrite(ENA_SPECTRO, LOW);//turns off

  digitalWrite(LED_POWER, HIGH);      //turns on
  digitalWrite(LED_HRTBT, LOW);      //turns off
  digitalWrite(LED_SDACTIVE, LOW);  //turns off
  digitalWrite(LED_ERROR, LOW);    //turns off
  digitalWrite(LED_COMM, LOW);    //turns off

  //Interupt 
  attachInterrupt(TE2_ISOLATED, te2, CHANGE); // The interrupt is a program that occurs when TE2 changes telling the computer to watch when to begin the TE2 program (this TE2 program is a function)
  // Start I2C
  Wire.begin(); // I2C is connected
  //Setup SD Card
}

void loop() {
  // put your main code here, to run repeatedly:
  //Check to see if TE-2 is pulled high

}

void te2(){
  //Runs when TE-2 is triggered
  //Sets the enable to true
  sys.status.enable_iridium = true; // the iridium runs 
  sys.status.enable_spectrometer = true;
  digitalWrite(ENA_SPECTRO, HIGH); //Spectrometer is turned on
  digitalWrite(ENA_IRIDIUM, HIGH); // Iridium is active
}

void saveData(String filename){
  File file = SD.open(filename, FILE_WRITE); //This (SD card) reads and writes the data to a .txt file
  if(file){
    unsigned char headerBytes[4] = {'O','D','I','N'}; // Header in the .txt file
    unsigned char footerBytes[4] = {'$','$','o','7'}; // Footer in the .txt file

    uint8_t checksum = 0; //
    uint8_t* ptr = (uint8_t*)&sys;  //
    for (size_t i = 0; i <sizeof(sys); i++){ // 
      checksum +=ptr[i];
    }

    file.write(headerBytes, sizeof(headerBytes)); //
    size_t bytesWritten = file.write((const uint8_t *)&sys, sizeof(sys));
    file.write(&checksum,1);
    file.write(footerBytes, sizeof(footerBytes));
    file.flush();
    file.close(); //file closes
  }
}

void sendToIridium(){

  //The FSW takes in data, write to SDcard and sending the .txt to iridium
  
  //iridium_send_cmd("")
  //File file = SD.open(filename, FILE_READ); //This reads the file that the SD card contained for sending to the Iridium


  //if data sent
  //display("Data has been sent to the Iridium");
  //std::map<std::string, int> ODinDictionary; //This creates a dictionary to identify the status of each component in the system
  //ODinDictionary["Iridium"] = 0;
  //ODinDictionary["COMMS"] = 1;
  //ODinDictionary["EPDS"] = 2;
  //ODinDictionary["FSW"] = 3;
  //ODinDictionary["AI"] = 4;
  //ODinDictionary["INST"] = 5;
  //else 
   // display("Data not sent to Iridium (error)");
};
