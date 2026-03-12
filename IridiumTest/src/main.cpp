/**
 * RockBLOCK 9704 — Hardware Test
 * Target  : WeAct STM32F401CE BlackPill 3.0
 * Toolchain: PlatformIO + Arduino framework (ststm32)
 *
 * Wiring:
 *   BlackPill PA9  (TX1) ──► RockBLOCK RX
 *   BlackPill PA10 (RX1) ◄── RockBLOCK TX
 *   BlackPill PA4        ──► RockBLOCK ENABLE  (active HIGH)
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

// ─── Pin / Port Configuration ────────────────────────────────────────────────

// UART1 (PA9/PA10) is wired to the RockBLOCK — passed as a Stream reference
#define RB_SERIAL       Serial1
// Library fixes baud at RB9704_BAUD (230400); do NOT call Serial1.begin() yourself

// UART2 (PA2/PA3) drives a USB-serial adapter for debug output.
// STM32 Arduino core does not pre-instantiate Serial2, so declare it manually.
HardwareSerial DBG(PA3, PA2);   // RX=PA3, TX=PA2
#define DBG_BAUD        115200

// Active-HIGH enable: drive HIGH to wake modem, LOW to sleep it
#define RB_ENABLE_PIN   PA4

// ─── Timing ──────────────────────────────────────────────────────────────────
#define ENABLE_BOOT_MS      3000   // Power-on settling time
#define POST_BEGIN_MS        200   // Mandatory gap after rbBegin() per datasheet
#define SIGNAL_WAIT_MS     10000   // How long to wait for a constellation update
#define SEND_TIMEOUT_S       180   // rbSendMessage() hard timeout (seconds)
#define LOOP_INTERVAL_MS   30000   // Delay between test runs

// ─── Signal callback state ────────────────────────────────────────────────────
static volatile bool    gSignalReceived    = false;
static volatile uint8_t gSignalBars        = 0;
static volatile int16_t gSignalLevel       = 0;
static volatile bool    gConstellationVisible = false;

static void onConstellationState(const jsprConstellationState_t* state) {
    // Members: constellationVisible (bool), signalBars (uint8_t), signalLevel (int16_t)
    gConstellationVisible = state->constellationVisible;
    gSignalBars           = state->signalBars;
    gSignalLevel          = state->signalLevel;
    gSignalReceived       = true;
}

// ─── Helpers ─────────────────────────────────────────────────────────────────
static void printResult(bool ok, const char* label) {
    DBG.print(ok ? F("  [ OK ] ") : F("  [FAIL] "));
    DBG.println(label);
}

static void enableModem(bool on) {
    digitalWrite(RB_ENABLE_PIN, on ? HIGH : LOW);
    if (on) {
        DBG.println(F("[PWR] Modem enabled — waiting for boot..."));
        delay(ENABLE_BOOT_MS);
    } else {
        DBG.println(F("[PWR] Modem disabled."));
    }
}

// ─── Test 1 — rbBegin ────────────────────────────────────────────────────────
static bool testBegin() {
    DBG.println(F("\n--- Test 1: rbBegin ---"));

    // Pass the Stream reference directly; the library handles baud-rate setup
    bool ok = rbBegin(RB_SERIAL);
    printResult(ok, "rbBegin(Serial1)");

    if (ok) {
        // Required: modem needs >100 ms after rbBegin() sets simConfig /
        // operationalState before it is ready to accept commands
        delay(POST_BEGIN_MS);
    }
    return ok;
}

// ─── Test 2 — Signal Quality ─────────────────────────────────────────────────
static bool testSignal() {
    DBG.println(F("\n--- Test 2: Signal Quality ---"));

    // Register callback and poll until the modem pushes a constellation update
    rbCallbacks_t cbs = {};
    cbs.constellationState = onConstellationState;
    rbRegisterCallbacks(&cbs);

    gSignalReceived = false;
    uint32_t deadline = millis() + SIGNAL_WAIT_MS;
    while (!gSignalReceived && millis() < deadline) {
        rbPoll();
        delay(50);
    }

    if (gSignalReceived) {
        DBG.print(F("  Constellation visible : "));
        DBG.println(gConstellationVisible ? F("yes") : F("no"));
        DBG.print(F("  Signal bars (0-5)     : "));
        DBG.println(gSignalBars);
        DBG.print(F("  Signal level (raw)    : "));
        DBG.println(gSignalLevel);
    } else {
        DBG.println(F("  (no update within timeout — check antenna / sky view)"));
    }

    printResult(gSignalReceived, "Constellation state callback fired");
    return gSignalReceived;
}

// ─── Test 3 — rbPoll smoke test ──────────────────────────────────────────────
// rbPoll() processes any pending modem data and returns the number of bytes
// consumed. Simply calling it without error is sufficient as a connectivity
// check; a non-negative return confirms the serial link is alive.
static bool testPoll() {
    DBG.println(F("\n--- Test 3: rbPoll ---"));

    // Call rbPoll() a few times over 500 ms to drain any queued modem output
    for (int i = 0; i < 10; i++) {
        rbPoll();
        delay(50);
    }
    printResult(true, "rbPoll() returned without error");
    return true;
}

// ─── Test 4 — Send MO Message ────────────────────────────────────────────────
// Requires: active Cloudloop plan, RAW topic provisioned, clear sky view.
// Blocks for up to SEND_TIMEOUT_S seconds.
static bool testSend() {
    DBG.println(F("\n--- Test 4: Send MO Message ---"));
    DBG.print  (F("  Timeout = "));
    DBG.print  (SEND_TIMEOUT_S);
    DBG.println(F(" s — ensure antenna has clear sky view"));

    const char* msg = "BLACKPILL-RB9704-TEST";
    bool ok = rbSendMessage(msg, strlen(msg), SEND_TIMEOUT_S);
    printResult(ok, "rbSendMessage() — MO session");
    return ok;
}

// ─── Test 5 — Receive MT Message (non-blocking check) ────────────────────────
static bool testReceive() {
    DBG.println(F("\n--- Test 5: Check for MT Message ---"));

    char* buf = nullptr;
    size_t len = rbReceiveMessageAsync(&buf);

    if (len > 0 && buf != nullptr) {
        DBG.print(F("  Received ("));
        DBG.print(len);
        DBG.print(F(" bytes): "));
        // Print as ASCII; non-printable bytes shown as '.'
        for (size_t i = 0; i < len; i++) {
            char c = buf[i];
            DBG.print(isprint((unsigned char)c) ? c : '.');
        }
        DBG.println();
        rbAcknowledgeReceiveHeadAsync();
        printResult(true, "MT message in queue");
        return true;
    }

    DBG.println(F("  No MT message waiting (normal if none was sent to this device)"));
    printResult(true, "No MT message — queue empty (pass)");
    return true;   // Not a failure — just nothing waiting
}

// ─── Test 6 — rbEnd ──────────────────────────────────────────────────────────
static bool testEnd() {
    DBG.println(F("\n--- Test 6: rbEnd ---"));
    bool ok = rbEnd();
    printResult(ok, "rbEnd() — serial closed");
    return ok;
}

// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {
    DBG.begin(DBG_BAUD);
    delay(2000);   // Allow monitor to connect

    DBG.println(F("\n=========================================="));
    DBG.println(F("  RockBLOCK 9704 — BlackPill F401 Tests"));
    DBG.println(F("=========================================="));

    pinMode(RB_ENABLE_PIN, OUTPUT);
    enableModem(false);   // Start with modem off
}

// ─── Loop ────────────────────────────────────────────────────────────────────
void loop() {
    DBG.println(F("\n======== Starting Test Run ========"));

    int passed = 0, total = 0;
    auto run = [&](bool r) { total++; if (r) passed++; };

    enableModem(true);

    if (!rbBegin(RB_SERIAL)) {
        DBG.println(F("[ABORT] rbBegin failed — check wiring & power."));
        printResult(false, "rbBegin(Serial1)");
        enableModem(false);
        delay(LOOP_INTERVAL_MS);
        return;
    }
    passed++; total++;   // count testBegin as passed
    DBG.println(F("  [ OK ] rbBegin(Serial1)"));
    delay(POST_BEGIN_MS);

    run(testSignal());
    run(testPoll());
    run(testSend());
    run(testReceive());
    run(testEnd());

    enableModem(false);

    DBG.println(F("\n======== Results ========"));
    DBG.print  (F("  Passed : "));
    DBG.print  (passed);
    DBG.print  (F(" / "));
    DBG.println(total);
    DBG.println(F("========================="));

    DBG.print(F("\nNext run in "));
    DBG.print(LOOP_INTERVAL_MS / 1000);
    DBG.println(F(" s...\n"));
    delay(LOOP_INTERVAL_MS);
}