#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <SPI.h>
#include <STM32RTC.h>
#include "pins.h"


//Globals
JsonDocument doc;
STM32RTC rtc;

//Prototypes
void te2(void);


void setup() {
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

  // Start I2C
  Wire.begin();

  //Setup SD Card

}

void loop() {
  // put your main code here, to run repeatedly:

  //Check to see if TE-2 is pulled high
  if(TE2_ISOLATED == HIGH){
    te2();
  }

}

void te2(){
  //Runs when TE-2 is triggered
  digitalWrite(ENA_SPECTRO, HIGH);
  digitalWrite(ENA_IRIDIUM, HIGH);
}
