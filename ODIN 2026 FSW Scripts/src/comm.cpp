#include <Arduino.h> // STANDARD ARDUINO LIBRARY
#include <Wire.h> // WIRE LIBRARY
#include <rockblock_9704.h> // IRIDIUM 9704 LIBRARY
#include <imt_queue.h> // IRIDIUM 9704 LIBRARY
#include <comm.h> // COMMS DEFINITIONS
#include <fsw.h> // FSW DEFINITIONS
#include <pins.h> // PINS DEFINITIONS

// -- INIT FUNCTIONS -- //
void clearQueue(){
    while(imtQueueMtGetFirst()){
        imtQueueMoRemove(); // Removes message in queue if one is found
    }
}

void initCOMMStatus(COMM &comm){
    comm.ENBL_STATUS = false; // COMMS are initially disabled
    comm.TX_ACTIVE = false; // No transmission is active at startup
    comm.messagesSent = 0; // Initialize messages sent counter to 0
}

bool initCOMM(COMM &comm){
    // Initialize COMMS pins
    digitalWrite(COMM_EN, HIGH); // Bring Iridium Enable Pin HIGH to enable COMMS XMIT

    unsigned long startTime = millis(); // Setup a timeout to prevent getting stuck if COMMS fails to enable
    const unsigned long timeout = 15000; // 15 second timeout for COMMS to enable

    while (digitalRead(COMM_BTD) == LOW) { // Wait for COMMS to enable (read 9704 Boot State)
        if (millis() - startTime > timeout) {
            DEBUG_SERIAL.println("[COMM] ERROR: COMMS failed to enable within timeout period.");
            return false; // Return false if COMMS fail to enable within the timeout period
        }
        delay(100); // Short delay to avoid sampling too quickly
    }

    if(!rbBegin(COMM_SERIAL)) { // Initialize RockBLOCK communication
        DEBUG_SERIAL.println("[COMM] ERROR: Failed to initialize COMMS Serial communication.");
        return false; // Return false if COMMS initialization fails
    }

    clearQueue();

    comm.ENBL_STATUS = true; // Update COMMS status to enabled
    return true; // Enable pin pulled high, COMMS should be enabled
}

bool commShutDown(COMM &comm){
    // Disable COMMS
    rbEnd(); // End RockBLOCK communication
    digitalWrite(COMM_EN, LOW); // Disable COMMS

    unsigned long startTime = millis(); // Setup a timeout to prevent getting stuck if COMMS fails to disable
    const unsigned long timeout = 15000; // 15 second timeout for COMMS to enable

    while (digitalRead(COMM_BTD) == HIGH) { // Wait for COMMS to disable (read 9704 Boot State)
        if (millis() - startTime > timeout) {
            DEBUG_SERIAL.println("[COMM] ERROR: COMMS failed to disable within timeout period.");
            return false; // Return false if COMMS fail to disable within the timeout period
        }
        delay(100); // Short delay to avoid sampling too quickly
    }

    comm.ENBL_STATUS = false; // Update COMMS status to disabled
    return true; // Return true to indicate successful shutdown
}

bool sendMessage(FSW &fsw, COMM &comm){
    if ((fsw.currentMissionTime - fsw.lastTransmit) >= COMM_SEND_INTERVAL){
        fsw.lastTransmit = now();
        DEBUG_SERIAL.println("[COMM] Attempting to send message at: " + String(fsw.lastTransmit));

        if (!comm.ENBL_STATUS) {
            DEBUG_SERIAL.println("[COMM] ERROR: Cannot send message, COMMS not enabled.");
            return false; // Cannot send message if COMMS are not enabled
        }

        clearQueue();

        String messageTemp;
        messageTemp = "a" + String(fsw.currentMissionTime) + "b" + fsw.epdsToSave + "c" + fsw.fswToSave + "d" + fsw.AIToSave;
        int messageLength = messageTemp.length();
        char message[messageLength];
        messageTemp.toCharArray(message, messageLength);
        
        int8_t signalStrength;
        signalStrength = rbGetSignal();
        DEBUG_SERIAL.println("[COMM] Attempting to send message with " + String(signalStrength) + " signal!");


        if(!rbSendMessageAny(244, message, messageLength, COMM_TIMEOUT)) { // Send current mission time as message payload (max 244 bytes for Iridium Short Burst Data)
            DEBUG_SERIAL.println("[COMM] ERROR: Failed to send message via COMMS.");
            return false; // Return false if message transmission fails
        }

        comm.messagesSent++; // Increment messages sent counter
        DEBUG_SERIAL.println("[COMM] Message sent: " + messageTemp);
        return true; // Return true to indicate successful transmission
    } 
}