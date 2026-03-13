/**
 * RockBLOCK 9704 — Hardware Test
 * Target  : WeAct STM32F401CE BlackPill 3.0
 * Toolchain: PlatformIO + Arduino framework (ststm32)
 *
 * Wiring:
 *   BlackPill PA9  (TX1) ──► RockBLOCK RX
 *   BlackPill PA10 (RX1) ◄── RockBLOCK TX
 *   BlackPill PA4        ──► RockBLOCK ENABLE  (active HIGH)
 *   BlackPill PB1        ◄── RockBLOCK BOOT_DONE (I_BTD)
 *   BlackPill PA2  (TX2) ──► USB-Serial RX  [debug]
 *   BlackPill PA3  (RX2) ◄── USB-Serial TX  [debug]
 *   BlackPill GND        ──► RockBLOCK GND
 *   5 V rail             ──► RockBLOCK VCC
 *
 * Library : rock7/RockBLOCK-9704 @ ^0.1.29
 *   rbBegin(Stream&)               — open session
 *   rbRegisterCallbacks(callbacks) — register event callbacks
 *   rbSendMessageAsync(topic, msg, len) — queue MO message
 *   rbPoll()                       — must be called frequently (~10ms)
 *   rbEnd()                        — close session gracefully
 */

#include <Arduino.h>
#include <rockblock_9704.h>

// ── Pin definitions ───────────────────────────────────────────────────────────
#define I_EN  PA4
#define I_BTD PB1

// ── Message topic ─────────────────────────────────────────────────────────────
#define TEST_TOPIC 244  // RB9704 must be provisioned for this topic

// ── Timeout constants ─────────────────────────────────────────────────────────
static const uint32_t BOOT_TIMEOUT_MS     = 30000;
static const uint32_t SHUTDOWN_TIMEOUT_MS = 10000;

// ── Serial ports ──────────────────────────────────────────────────────────────
HardwareSerial IridiumSerial(PA10, PA9);  // UART1 → RockBLOCK
HardwareSerial DebugSerial(PA3, PA2);     // UART2 → USB-serial adapter

// ── State ─────────────────────────────────────────────────────────────────────
static int messagesSent = 0;

// ── Forward declarations ──────────────────────────────────────────────────────
bool startupSequence();
bool shutdownSequence();
void onMessageProvisioning(const jsprMessageProvisioning_t* provisioning);
void onMoComplete(const uint16_t id, const rbMsgStatus_t status);
void onMtComplete(const uint16_t id, const rbMsgStatus_t status);
void onConstellationState(const jsprConstellationState_t* state);

// ── Callbacks ─────────────────────────────────────────────────────────────────
static rbCallbacks_t myCallbacks = {
    .messageProvisioning = onMessageProvisioning,
    .moMessageComplete   = onMoComplete,
    .mtMessageComplete   = onMtComplete,
    .constellationState  = onConstellationState
};

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    pinMode(I_EN,  OUTPUT);
    pinMode(I_BTD, INPUT);
    digitalWrite(I_EN, LOW);

    DebugSerial.begin(9600);
    DebugSerial.println(F("[INIT] Debug serial ready"));

    if (!startupSequence()) {
        DebugSerial.println(F("[ERROR] Startup timed out — halting"));
        while (true) { delay(1000); }
    }

    IridiumSerial.begin(230400);  // RB9704 baud rate
    DebugSerial.println(F("[INIT] Iridium serial opened at 230400"));

    if (!rbBegin(IridiumSerial)) {
        DebugSerial.println(F("[ERROR] rbBegin() failed — check wiring"));
        shutdownSequence();
        while (true) { delay(1000); }
    }

    rbRegisterCallbacks(&myCallbacks);
    DebugSerial.println(F("[INIT] RockBLOCK session started"));

    const char* message = "RockBLOCK 9704 hardware test";
    if (rbSendMessageAsync(TEST_TOPIC, message, strlen(message))) {
        DebugSerial.print(F("[TX] Queued: "));
        DebugSerial.println(message);
    } else {
        DebugSerial.println(F("[TX] Failed to queue message"));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    rbPoll();  // must be called frequently — drives all modem communication
    delay(10);

    if (messagesSent > 0) {
        if (rbEnd()) {
            DebugSerial.println(F("[DONE] Session ended successfully"));
            shutdownSequence();
            messagesSent = 0;
            while (true) { delay(1000); }  // test complete, halt
        } else {
            DebugSerial.println(F("[ERROR] rbEnd() failed"));
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
bool startupSequence() {
    DebugSerial.println(F("[PWR] Asserting I_EN …"));
    digitalWrite(I_EN, HIGH);

    uint32_t start = millis();
    while (digitalRead(I_BTD) == LOW) {
        if (millis() - start > BOOT_TIMEOUT_MS) {
            DebugSerial.println(F("[PWR] Timeout waiting for I_BTD HIGH"));
            digitalWrite(I_EN, LOW);
            return false;
        }
        delay(10);
    }

    DebugSerial.print(F("[PWR] Module ready in "));
    DebugSerial.print(millis() - start);
    DebugSerial.println(F(" ms"));
    return true;
}

bool shutdownSequence() {
    DebugSerial.println(F("[PWR] De-asserting I_EN …"));
    digitalWrite(I_EN, LOW);

    uint32_t start = millis();
    while (digitalRead(I_BTD) == HIGH) {
        if (millis() - start > SHUTDOWN_TIMEOUT_MS) {
            DebugSerial.println(F("[PWR] Timeout waiting for I_BTD LOW"));
            return false;
        }
        delay(10);
    }

    DebugSerial.println(F("[PWR] Module powered down"));
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
void onMessageProvisioning(const jsprMessageProvisioning_t* provisioning) {
    if (provisioning->provisioningSet) {
        DebugSerial.print(F("[PROV] Provisioned for "));
        DebugSerial.print(provisioning->topicCount);
        DebugSerial.println(F(" topic(s):"));
        for (int i = 0; i < provisioning->topicCount; i++) {
            DebugSerial.print(F("  Topic: "));
            DebugSerial.print(provisioning->provisioning[i].topicName);
            DebugSerial.print(F(" (#"));
            DebugSerial.print(provisioning->provisioning[i].topicId);
            DebugSerial.println(F(")"));
        }
    }
}

void onMoComplete(const uint16_t id, const rbMsgStatus_t status) {
    DebugSerial.print(F("[MO] ID="));
    DebugSerial.print(id);
    DebugSerial.print(F(" Status="));
    DebugSerial.println(status);
    if (status == RB_MSG_STATUS_OK) {
        messagesSent++;
        DebugSerial.println(F("[MO] Message confirmed sent"));
    }
}

void onMtComplete(const uint16_t id, const rbMsgStatus_t status) {
    DebugSerial.print(F("[MT] ID="));
    DebugSerial.print(id);
    DebugSerial.print(F(" Status="));
    DebugSerial.println(status);
}

void onConstellationState(const jsprConstellationState_t* state) {
    DebugSerial.print(F("[SIG] Signal bars: "));
    DebugSerial.println(state->signalBars);
}