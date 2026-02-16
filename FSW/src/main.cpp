#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <STM32RTC.h>
#include "rockblock_9704.h"
#include "pins.h"
#include "data.h"

//DEFINES
#define MIN_TE2_WAIT 10    // Wiat at least 10 seconds before arming TE-2, just to make sure the payload isn't triggered early when starting


//Globals
JsonDocument doc;
FSW_SYSTEM sys;
STM32RTC& rtc = STM32RTC::getInstance();


//Prototypes
void te2(void);


void setup() {
  //Set Mission Start time
  sys.mission_start.setFromRTC(rtc);

  //Setup Pins
  pinMode(TE2_ISOLATED, INPUT);
  pinMode(ENA_IRIDIUM, OUTPUT);
  pinMode(ENA_SPECTRO, OUTPUT);
  pinMode(LED_POWER, OUTPUT);
  pinMode(LED_HRTBT, OUTPUT);
  pinMode(LED_SDACTIVE, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);
  pinMode(LED_COMM, OUTPUT);

  digitalWrite(ENA_IRIDIUM, LOW);
  digitalWrite(ENA_SPECTRO, LOW);

  digitalWrite(LED_POWER, HIGH);
  digitalWrite(LED_HRTBT, LOW);
  digitalWrite(LED_SDACTIVE, LOW);
  digitalWrite(LED_ERROR, LOW);
  digitalWrite(LED_COMM, LOW);

  //Interupt 
  attachInterrupt(TE2_ISOLATED, te2, CHANGE);
  // Start I2C
  Wire.begin();
  //Setup SD Card
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

    file.write(headerBytes, sizeof(headerBytes));
    size_t bytesWritten = file.write((const uint8_t *)&sys, sizeof(sys));
    file.write(&checksum,1);
    file.write(footerBytes, sizeof(footerBytes));
    file.flush();
    file.close();
  }
}

void sendToIridium(){
};