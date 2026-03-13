#include <Arduino.h>
#include <rockblock_9704.h>

HardwareSerial DBG(USART2); 

#define RB_SERIAL     Serial1
#define RB_ENABLE_PIN PA4  // I_EN
#define RB_BTD_PIN    PB1  // I_BTD (Boot/Traffic Detect)

void setup() {
    DBG.begin(115200);
    
    // Initial state: Everything Tristate/Input per datasheet startup rule #1
    pinMode(RB_ENABLE_PIN, INPUT); 
    pinMode(RB_BTD_PIN, INPUT);
    pinMode(PA9, INPUT); // Serial TX pin Tristate
    pinMode(PA10, INPUT); // Serial RX pin Tristate

    DBG.println("System Initialized - RockBLOCK in Tristate/OFF");
}

bool startIridium() {
    DBG.println("[PWR] Starting Startup Sequence...");

    // 1. Tristate / Logic High I_EN (Internal pull-up usually handles this, but we drive it)
    pinMode(RB_ENABLE_PIN, OUTPUT);
    digitalWrite(RB_ENABLE_PIN, HIGH);

    // 2. Wait for I_BTD to go HIGH (The 'Modem is Booted' signal)
    DBG.println("[PWR] Waiting for I_BTD HIGH...");
    uint32_t timeout = millis() + 5000; 
    while (digitalRead(RB_BTD_PIN) == LOW) {
        if (millis() > timeout) {
            DBG.println("[FAIL] Startup Timeout - I_BTD stayed LOW");
            return false;
        }
        delay(10);
    }

    // 3. UART / Input pins can now be initialized
    if (rbBegin(RB_SERIAL)) {
        DBG.println("[OK] Modem Booted and Serial Active");
        return true;
    }
    return false;
}

void shutdownIridium() {
    DBG.println("[PWR] Starting Shutdown Sequence...");

    // 1. Cease serial communications
    rbEnd();

    // 2. Drive I_EN LOW
    digitalWrite(RB_ENABLE_PIN, LOW);

    // 3. Wait for I_BTD to go LOW
    DBG.println("[PWR] Waiting for I_BTD LOW...");
    uint32_t timeout = millis() + 5000;
    while (digitalRead(RB_BTD_PIN) == HIGH) {
        if (millis() > timeout) break; // Don't hang forever
        delay(10);
    }

    // 4. Set all pins to tristate (High-Z)
    pinMode(PA9, INPUT);
    pinMode(PA10, INPUT);
    pinMode(RB_ENABLE_PIN, INPUT); 

    DBG.println("[OFF] Modem Safe to Power Down.");
}

void loop() {
    if (startIridium()) {
        // Run your tests here
        delay(5000); 
        shutdownIridium();
    }
    
    delay(30000); // Wait for next cycle
}