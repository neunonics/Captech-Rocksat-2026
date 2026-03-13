/**
 * RockBLOCK 9704 — Hardware Test
 * Target  : WeAct STM32F401CE BlackPill 3.0
 * Toolchain: PlatformIO + Arduino framework (ststm32)
 *
 * Wiring:
 *   BlackPill PA9  (TX1) ──► RockBLOCK RX
 *   BlackPill PA10 (RX1) ◄── RockBLOCK TX
 *   BlackPill PA4        ──► RockBLOCK ENABLE  (active HIGH)
 *   Blackpill PB0
 *   BlackPill GND        ──► RockBLOCK GND
 *   5 V rail             ──► RockBLOCK VCC
 *
 *   Debug output uses Serial (UART2 / PA2-PA3) via a USB-serial adapter.
 *
 * Library : rock7/RockBLOCK-9704 @ ^0.1.29
 *   Arduino-variant API differences vs the Linux C build:
 *     rbBegin(Stream &port)   — pass a Stream reference, NOT a string
 *     Hardware-info / SIM-status helpers are NOT present in the Arduino build;
 *     signal quality uses the rbCallbacks_t::constellationState callback.
 */

#include <Arduino.h>
#include <rockblock_9704.h>

#define I_EN PA4
#define I_BTD PB1

HardwareSerial IRIDIUM(PA10, PA9);
HardwareSerial DEBUSERIAL(PA3, PA2);

//ROCKBLOCK_9704_H rb;

void SendMessage(const char*, size_t);
void StartupSequence();
void ShutdownSequence();

void setup(){
    pinMode(I_EN, OUTPUT);
    pinMode(I_BTD, INPUT);
    digitalWrite(I_EN, LOW);

    IRIDIUM.begin(19200);

    StartupSequence();
    rbBegin(IRIDIUM);


}

void StartupSequence(){
//Set all pins low
// Apply power to rockblock
//Pull I_EN High
digitalWrite(I_EN, HIGH);
// Wait for I_BTD to be high
while (digitalRead(I_BTD) == LOW){
    delay(10);
}
}

void ShutdownSequence(){
//Drive I_EN Low
digitalWrite(I_EN, LOW);
// Wait for I_BTD to go low

while (digitalRead(I_BTD) == HIGH){
    delay(10);
}
//Set all pins low
}

void SendMessage(const char* msg, size_t len){

    int result = rbSendMessage(msg, len, 1000);
}

void loop(){

}