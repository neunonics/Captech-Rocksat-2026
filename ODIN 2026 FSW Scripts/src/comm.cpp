#include <Arduino.h> // STANDARD ARDUINO LIBRARY
#include <Wire.h> // WIRE LIBRARY
#include <rockblock_9704.h> // IRIDIUM 9704 LIBRARY
#include <comm.h> // COMMS DEFINITIONS
#include <pins.h> // PINS DEFINITIONS

// -- INIT FUNCTIONS -- //
void initCOMMStatus(COMM &comm){
    comm.ENBL_STATUS = false; // COMMS are initially disabled
    comm.TX_ACTIVE = false; // No transmission is active at startup
    comm.messagesSent = 0; // Initialize messages sent counter to 0
}

bool initCOMM(COMM &comm){
    // Initialize COMMS pins
    digitalWrite(COMM_EN, HIGH); // Enable PIN High to enable COMMS

    unsigned long startTime = millis(); // Setup a timeout to prevent getting stuck if COMMS fail to enable
    const unsigned long timeout = 10000; // 10 second timeout for COMMS to enable

    while (digitalRead(COMM_BTD) == LOW) { // Wait for COMMS to enable
        if (millis() - startTime > timeout) {
            Serial.println("ERROR: COMMS failed to enable within timeout period.");
            return false; // Return false if COMMS fail to enable within the timeout period
        }
        delay(100); // Short delay to avoid busy-waiting
    }

    comm.ENBL_STATUS = true; // Update COMMS status to enabled
    return true; // Enable pin pulled high, COMMS should be enabled
}

bool commShutDown(COMM &comm){
    // Disable COMMS
    rbEnd(); // End RockBLOCK communication
    digitalWrite(COMM_EN, LOW); // Disable COMMS

    unsigned long startTime = millis(); // Setup a timeout to prevent getting stuck if COMMS fail to enable
    const unsigned long timeout = 10000; // 10 second timeout for COMMS to enable

    while (digitalRead(COMM_BTD) == HIGH) { // Wait for COMMS to disable
        if (millis() - startTime > timeout) {
            Serial.println("ERROR: COMMS failed to disable within timeout period.");
            return false; // Return false if COMMS fail to disable within the timeout period
        }
        delay(100); // Short delay to avoid busy-waiting
    }

    comm.ENBL_STATUS = false; // Update COMMS status to disabled
    return true; // Return true to indicate successful shutdown
}