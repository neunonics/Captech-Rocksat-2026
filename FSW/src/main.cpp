#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <STM32RTC.h>
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
  sys.mission_start.day = rtc.getDay();
  sys.mission_start.hour = rtc.getHours();
  sys.mission_start.minute = rtc.getMinutes();
  sys.mission_start.month = rtc.getMonth();
  sys.mission_start.year = rtc.getYear();
  sys.mission_start.second = rtc.getSeconds();
  sys.mission_start.subseconds = rtc.getSubSeconds();

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

  attachInterrupt(TE2_ISOLATED, te2, RISING);
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
