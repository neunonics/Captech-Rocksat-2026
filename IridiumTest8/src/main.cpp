/**
 * RockBLOCK 9704 — Startup/Shutdown Toggle Test Script
 * Target  : WeAct STM32F401CE BlackPill 3.0
 * Toolchain: PlatformIO + Arduino framework (ststm32)
 *
 * Wiring:
 *   BlackPill PA9  (TX1) ──► RockBLOCK 14 (RX)
 *   BlackPill PA10 (RX1) ◄── RockBLOCK 13 (TX)
 *   BlackPill PA4        ──► RockBLOCK 3 (ENABLE, active HIGH)

 *   BlackPill GND        ──► RockBLOCK 1, 4, 10, 16 (Signal, Power GND)
 *   5 V rail             ──► RockBLOCK 15 (5V Power In)
 *
 *   Debug output uses Serial (UART2 / PA2-PA3) via a USB-serial adapter.
 *
 * Library : rock7/RockBLOCK-9704 @ ^0.1.29
 *   Arduino-variant API differences vs the Linux C build:
 *     rbBegin(Stream &port)   — pass a Stream reference, NOT a string
 *     Hardware-info / SIM-status helpers are NOT present in the Arduino build;
 *     signal quality uses the rbCallbacks_t::constellationState callback.
 **/

// Include necessary libraries and headers
#include <Arduino.h>
#include <rockblock_9704.h>

// Define pin assignments
#define I_EN PA4
#define I_BTD PB1

// Define serial assignments
HardwareSerial IRIDIUM(PA10, PA9);
HardwareSerial DEBUGSERIAL(PA3, PA2);

bool queuedMsgSent = false; // Represents if a queued message has been sent

// Define functions before setup() and loop() for better organization

// Callback function for message sent status
void onMessageSent(const uint16_t id, const rbMsgStatus_t status) {
    // status == 1 means success
    if (status == 1) {
        queuedMsgSent = true;
    }
}

extern "C" void HardFault_Handler(void) {
    DEBUGSERIAL.println("HARD FAULT");
    while(1){
        digitalWrite(PC13, HIGH);
        delay(200);
        digitalWrite(PC13, LOW);
        delay(200);
    }
}

// Define the callback structure for the RockBLOCK library
rbCallbacks_t myCallbacks = {
    .moMessageComplete = onMessageSent,
};


// 9704 power-up sequence based on vendor specs
void StartupSequence(){
    digitalWrite(I_EN, HIGH);
    DEBUGSERIAL.println("I_EN set HIGH");
    DEBUGSERIAL.print("I_BTD state immediately after: ");
    DEBUGSERIAL.println(digitalRead(I_BTD));
    
    unsigned long startTime = millis();
    const unsigned long timeout = 20000;
    
    while (digitalRead(I_BTD) == LOW){
        if (millis() - startTime > timeout) {
            DEBUGSERIAL.println("ERROR: timeout");
            break;
        }
        if ((millis() - startTime) % 1000 == 0){
            DEBUGSERIAL.print("Waiting... I_BTD: ");
            DEBUGSERIAL.println(digitalRead(I_BTD));
        }
        delay(10);
    }
    
    if (digitalRead(I_BTD) == HIGH) {
        DEBUGSERIAL.println("RockBLOCK module powered up successfully");
    }
}

// 9704 shut-down sequence based on vendor specs
void ShutdownSequence(){
    rbEnd(); // Ensure the library is properly closed before cutting power
    // Pull I_EN Low to power down the module
    digitalWrite(I_EN, LOW);
    
    // Wait for I_BTD to go low (module power down)
    unsigned long startTime = millis();
    const unsigned long timeout = 20000; // 20 second timeout
    
    while (digitalRead(I_BTD) == HIGH){
        if (millis() - startTime > timeout) {
            DEBUGSERIAL.println("WARNING: RockBLOCK did not power down within timeout");
            DEBUGSERIAL.println("WARNING: Proceed with caution, remove power manually if necessary");
            break;
        }
        delay(10);
    }
    
    if (digitalRead(I_BTD) == LOW) {
        DEBUGSERIAL.println("RockBLOCK module powered down successfully");
    }
}

// Function to send a synchronous message using the RockBLOCK library
void SendMessage(const char* msg, size_t len){
    int sendAttempt = rbSendMessage(msg, len, 1000);
}

// Arduino setup function
void setup(){
    IRIDIUM.begin(230400); // 230400 baud is the recommended baud rate for the RB 9704
    DEBUGSERIAL.begin(115200);
    pinMode(PC13, OUTPUT);

    for(int i = 0; i < 10; i++){
        digitalWrite(PC13, LOW);  // LED on
        delay(200);
        digitalWrite(PC13, HIGH); // LED off
        delay(200);
    }

    // Initialize pins
    pinMode(I_EN, OUTPUT);
    pinMode(I_BTD, INPUT_PULLDOWN);
    //
    DEBUGSERIAL.println("THIS IS A TEST");
    digitalWrite(I_EN, LOW);

    
    delay(500);
    
    
    //IRIDIUM.setRx(PA3);
    //IRIDIUM.setTx(PA2);
    //IRIDIUM.begin(115200);

    // Initialize debug serial for monitoring
    
    DEBUGSERIAL.println("A");
    //digitalWrite(PC13, LOW);
    
     // Power up the RockBLOCK module
    StartupSequence();

    // Initialize Iridium serial communication
    
    
    // Initialize RockBLOCK library with callbacks
    rbRegisterCallbacks(&myCallbacks);
    rbBegin(IRIDIUM);
    
    
    // Print startup message to debug serial
    DEBUGSERIAL.println("RockBLOCK 9704 verification script started");
}

int loopIteration = 0;
bool moduleOn = true;
// Loop function to trigger message sending and polling
void loop(){

    // Poll the RockBLOCK module for status updates
    rbPoll();
     // Only send a new message if the previous one has been sent
     static unsigned long lastAction = 0;
    if (millis() - lastAction >= 10000) {
        DEBUGSERIAL.println("HI");
        loopIteration++;
        if(!queuedMsgSent){
            const char* msg = "Hi, this is an interface test between FSW and COMM";
            size_t len = strlen(msg);
            
            // Send message asynchronously
            int sendAttempt = rbSendMessageAsync(0, msg, len);
            DEBUGSERIAL.println(sendAttempt);
            if (sendAttempt == 1){
                queuedMsgSent = true;
                DEBUGSERIAL.println("Message queued for transmission");
            } else {
                DEBUGSERIAL.println("Failed to queue message");
                DEBUGSERIAL.println("Initiating 9704 shutdown sequence");
                ShutdownSequence();
                // Exit the loop if we fail to queue a message
            }

            
        }
        lastAction = millis();
        
    } 
    // Make a small delay to prevent excessive polling */
    
}
