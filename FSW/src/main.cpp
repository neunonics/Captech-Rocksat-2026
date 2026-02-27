#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <STM32RTC.h>
#include "pins.h"
#include "data.h"

//DEFINES
#define MIN_TE2_WAIT 10    // Wiat at least 10 seconds before arming TE-2, just to make sure the payload isn't triggered early when starting


//Globals (Variables)
JsonDocument doc; //Variable
FSW_SYSTEM sys; //Variable
STM32RTC& rtc = STM32RTC::getInstance(); //Variable 


//Prototypes
void te2(void); //


void setup() {
  //Set Mission Start time
  sys.mission_start.setFromRTC(rtc); //The mission start time is set from RTC

  //Setup Pins (prepares pins to be either inputs or outputs)
  pinMode(TE2_ISOLATED, INPUT); //The Timed event (TE) The TE2 activates the iridium and the spectrometers
  pinMode(ENA_IRIDIUM, OUTPUT); //The flight computer outputs the signal to the iridium and spectrometer 
  pinMode(ENA_SPECTRO, OUTPUT); // output signal to the ENA_Spectro (spectrometer)
  pinMode(LED_POWER, OUTPUT); //  The LED power indicates that the whole circuit is on
  pinMode(LED_HRTBT, OUTPUT); //LED heartbeat 
  pinMode(LED_SDACTIVE, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);
  pinMode(LED_COMM, OUTPUT);

  digitalWrite(ENA_IRIDIUM, LOW); //pins are off in this section 
  digitalWrite(ENA_SPECTRO, LOW); //pins are off

  digitalWrite(LED_POWER, HIGH);      //This is turned on
  digitalWrite(LED_HRTBT, LOW);      //pins are off
  digitalWrite(LED_SDACTIVE, LOW);  //pins are off
  digitalWrite(LED_ERROR, LOW);    //pins are off
  digitalWrite(LED_COMM, LOW);    //pins are off

  //Interupt 
  attachInterrupt(TE2_ISOLATED, te2, CHANGE); // This takes the TE2 signal, telling the computer to watch when to begin the TE2 program (this TE2 program is a function)
  // Start I2C
  Wire.begin(); //tells the computer to start the I2C
  //Setup SD Card

  //save data file function


}

void loop() {
  // put your main code here, to run repeatedly:
  //Check to see if TE-2 is pulled high

}

void te2(){
  //Runs when TE-2 is triggered
  //Sets the enable to true
  sys.status.enable_iridium = true;
  sys.status.enable_spectrometer = true;
  digitalWrite(ENA_SPECTRO, HIGH);
  digitalWrite(ENA_IRIDIUM, HIGH);
}

void saveData(String filename){
  File file = SD.open(filename, FILE_WRITE);
  if(file){
    unsigned char headerBytes[4] = {'O','D','I','N'};
    unsigned char footerBytes[4] = {'$','$','o','7'};

    uint8_t checksum = 0;
    uint8_t* ptr = (uint8_t*)&sys;
    for (size_t i = 0; i <sizeof(sys); i++){
      checksum +=ptr[i];
    }

    //This is the block of code that prepares the SD card
    file.write(headerBytes, sizeof(headerBytes));
    size_t bytesWritten = file.write((const uint8_t *)&sys, sizeof(sys));
    file.write(&checksum,1);
    file.write(footerBytes, sizeof(footerBytes));
    file.flush();
    file.close();
  }
}