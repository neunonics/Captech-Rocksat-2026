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
HardwareSerial IRIDIUM(USART2);
HardwareSerial DEBUGSERIAL(PA10, PA9);

bool queuedMsgSent = false; // Represents if a queued message has been sent

// Define functions before setup() and loop() for better organization

// Callback function for message sent status
void onMessageSent(const uint16_t topic, rbMsgStatus_t status) {
    // status == 1 means success
    if (status == 1) {
        queuedMsgSent = false;
    }
}

// Define the callback structure for the RockBLOCK library
rbCallbacks_t myCallbacks = {
    .moMessageComplete = onMessageSent,
};


// 9704 power-up sequence based on vendor specs
void StartupSequence(){
    // Pull I_EN High
    digitalWrite(I_EN, HIGH);
    
    // Wait for I_BTD to be high (module ready)
    unsigned long startTime = millis();
    const unsigned long timeout = 20000; // 20 second timeout
    
    while (digitalRead(I_BTD) == LOW){
        if (millis() - startTime > timeout) {
            DEBUGSERIAL.println("ERROR: RockBLOCK failed to power up within timeout");
            break;
        }
        delay(10);
    }
    
    if (digitalRead(I_BTD) == HIGH) {
        DEBUGSERIAL.println("RockBLOCK module powered up successfully");
    }
}

// 9704 shut-down sequence based on vendor specs
void ShutdownSequence(){
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
    
    // Initialize pins
    pinMode(I_EN, OUTPUT);
    pinMode(I_BTD, INPUT_PULLDOWN);
    pinMode(PC13, OUTPUT);
    digitalWrite(I_EN, LOW);

    DEBUGSERIAL.begin(115200);
    delay(10);

    if (digitalRead(I_BTD) == HIGH) {
        DEBUGSERIAL.println("RB9704 already on");
        ShutdownSequence();
    }
    
    
    //IRIDIUM.setRx(PA3);
    //IRIDIUM.setTx(PA2);
    //IRIDIUM.begin(115200);

    // Initialize debug serial for monitoring
    
    DEBUGSERIAL.println("A");
    digitalWrite(PC13, LOW);
    
     // Power up the RockBLOCK module
    StartupSequence();

    // Initialize Iridium serial communication
    IRIDIUM.begin(230400); // 230400 baud is the recommended baud rate for the RB 9704
    
    // Initialize RockBLOCK library with callbacks
    rbBegin(IRIDIUM);
    rbRegisterCallbacks(&myCallbacks);
    
    // Print startup message to debug serial
    DEBUGSERIAL.println("RockBLOCK 9704 verification script started");
}

int loopItteration = 0;
bool moduleOn = true;
// Loop function to trigger message sending and polling
void loop(){
    delay(10000);
    /* if(digitalRead(I_BTD) == HIGH){
        digitalWrite(I_EN, LOW);
    }
    else{
        digitalWrite(I_EN, HIGH);
    } */
    DEBUGSERIAL.println("HI");
    // Poll the RockBLOCK module for status updates
    rbPoll();
    /* if (digitalRead(I_BTD) == HIGH) {
        ShutdownSequence();
    } */
/*     else if(digitalRead(I_BTD) == LOW){
        StartupSequence();
        rbBegin(IRIDIUM);
        rbRegisterCallbacks(&myCallbacks);
    } */
     // Only send a new message if the previous one has been sent
    if(!queuedMsgSent && loopItteration > 5){
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
    // Make a small delay to prevent excessive polling */
    loopItteration++;
}
