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
 *   Debug output uses DEBUG (UART2 / PA2-PA3) via a USB-serial adapter.
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
HardwareSerial IRIDIUM(USART1); // Iridium communication
HardwareSerial DEBUG(USART2);

// Function prototypes
void StartupSequence();
void ShutdownSequence();

// 9704 power-up sequence based on vendor specs
void StartupSequence(){
    // Pull I_EN High
    digitalWrite(I_EN, HIGH);
    
    // Wait for I_BTD to be high (module ready)
    unsigned long startTime = millis();
    const unsigned long timeout = 20000; // 20 second timeout
    
    while (digitalRead(I_BTD) == LOW){
        if (millis() - startTime > timeout) {
            DEBUG.println("ERROR: RockBLOCK failed to power up within timeout");
            return;
        }
        delay(10);
    }
    
    if (digitalRead(I_BTD) == HIGH) {
        DEBUG.println("RockBLOCK module powered up successfully");
    }
}

// 9704 shut-down sequence based on vendor specs
void ShutdownSequence(){
    // First, properly close the RockBLOCK library
    if (!rbEnd()) {
        DEBUG.println("WARNING: rbEnd() failed during shutdown");
    }
    
    // Pull I_EN Low to power down the module
    digitalWrite(I_EN, LOW);
    
    // Wait for I_BTD to go low (module power down)
    unsigned long startTime = millis();
    const unsigned long timeout = 20000; // 20 second timeout
    
    while (digitalRead(I_BTD) == HIGH){
        if (millis() - startTime > timeout) {
            DEBUG.println("WARNING: RockBLOCK did not power down within timeout");
            DEBUG.println("WARNING: Proceed with caution, remove power manually if necessary");
            return;
        }
        delay(10);
    }
    
    if (digitalRead(I_BTD) == LOW) {
        DEBUG.println("RockBLOCK module powered down successfully");
    }
}

static rbCallbacks_t myCallbacks =
{
/*.messageProvisioning = onMessageProvisioning,
.moMessageComplete = onMoComplete,
.mtMessageComplete = onMtComplete,
.constellationState = onConstellationState
*/};

// Arduino setup function
void setup(){
    // Initialize pins first
    pinMode(I_EN, OUTPUT);
    pinMode(I_BTD, INPUT_PULLDOWN);
    pinMode(PC13, OUTPUT);
    digitalWrite(I_EN, LOW); // Start with module powered off
    // Initialize debug serial for monitoring FIRST
    DEBUG.begin(9600);
    DEBUG.println("RockBLOCK 9704 verification script started");
    DEBUG.println("Initializing system...");


    delay(100);

    digitalWrite(PC13, LOW);

    // Check if RockBLOCK module is already powered on
    if (digitalRead(I_BTD) == HIGH) {
        DEBUG.println("WARNING: RockBLOCK module appears to be already powered on.");
        DEBUG.println("WARNING: Proceeding with caution...");
        digitalWrite(I_EN, LOW); // Attempt to power it down first
        // Don't immediately power it off, let the test continue
    }
    
    delay(100);

    // Power up the RockBLOCK module
    DEBUG.println("Starting RockBLOCK module...");
    delay(100);
    StartupSequence();

    // Initialize Iridium serial communication
    DEBUG.println("Initializing Iridium serial communication...");
    IRIDIUM.setTx(PA9);
    IRIDIUM.setRx(PA10);
    IRIDIUM.begin(230400, SERIAL_8N1); // Explicitly set pins for IRIDIUM
    delay(120);
    
    // Check if rbBegin succeeds
    if (!rbBegin(IRIDIUM)) {
        DEBUG.println("ERROR: rbBegin() failed — check wiring and power");
        DEBUG.println("ERROR: Cannot proceed with test");
        return;
    }
    
    DEBUG.println("RockBLOCK library initialized successfully");
    DEBUG.println("ONOFF interface test ready to proceed");
}

// Arduino loop function - for continuous testing if needed
void loop(){
    // This is a verification script, so we can add periodic checks here
    // or leave it empty for manual testing
    
    // Optional: Add a periodic status check
    static unsigned long lastStatusCheck = 0;
    if (millis() - lastStatusCheck > 5000) { // Check every 5 seconds
        lastStatusCheck = millis();
        DEBUG.print("Status: I_BTD = ");
        DEBUG.println(digitalRead(I_BTD));
    }
    
    delay(1000); // Small delay to prevent excessive CPU usage
}