#include <Arduino.h>
#include <rockblock_9704.h>

#define I_EN       PA4
#define I_BTD      PB1
#define TEST_TOPIC 244

HardwareSerial IridiumSerial(PA10, PA9);
HardwareSerial DebugSerial(PA3, PA2);

void setup() {
    DebugSerial.begin(9600);
    DebugSerial.println("Starting...");

    pinMode(I_EN,  OUTPUT);
    pinMode(I_BTD, INPUT);
    digitalWrite(I_EN, HIGH);

    DebugSerial.println("Waiting for I_BTD...");
    while (digitalRead(I_BTD) == LOW) { delay(10); }

    DebugSerial.println("I_BTD HIGH — settling 2s...");
    delay(2000);  // ← modem UART needs ~2s after I_BTD before it can talk
    DebugSerial.println("Ready");

    IridiumSerial.begin(230400);

    if (!rbBegin(IridiumSerial)) {
        DebugSerial.println("rbBegin FAILED");
        while (true) {}
    }
    DebugSerial.println("rbBegin OK — sending...");

    const char* msg = "RockBLOCK 9704 test";
    int result = rbSendMessage(TEST_TOPIC, msg, strlen(msg), 120000);

    DebugSerial.print("Send result: ");
    DebugSerial.println(result);  // 0 = success

    rbEnd();
    digitalWrite(I_EN, LOW);
    while (digitalRead(I_BTD) == HIGH) { delay(10); }
    DebugSerial.println("Done");
}

void loop() {}