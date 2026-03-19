#include <Arduino.h>
#include "gamma_serial.h"

// =============================================================================
// PIN / PORT CONFIGURATION — STM32F411CEU6 (Black Pill)
//
//   Serial1 (USART1)   — TX=PA9,  RX=PA10 — Spectrometer A
//   Serial2 (USART2)   — TX=PA2,  RX=PA3  — Spectrometer B
//   Serial6 (USART6)   — TX=PC6,  RX=PC7  — Combined spectrum downlink
//
// NOTE: OGD is 3.3V — Black Pill is also 3.3V, no level shifter needed.
// =============================================================================

// Black Pill onboard LED (active low on PC13)
#define LED_PIN               PC13

#define DOWNLINK_BAUD         115200
#define TRANSMIT_INTERVAL_MS  6000

// =============================================================================
// LED BLINK PATTERNS
//
// On power-up:          LED stays ON solid while setup() runs
// Setup done:           3 slow blinks  — code reached end of setup() OK
// Both detectors OK:    2 fast blinks  — GAMMA_Init() returned GAMMA_OK
// One detector missing: 1 slow blink   — partial init, continuing
// Both detectors dead:  rapid blink    — fatal halt
// Loop heartbeat:       1 very short pulse every 1 second — loop() is running
// TX fired:             2 short pulses — combined spectrum just transmitted
// =============================================================================


HardwareSerial Serial6(PA12, PA11);
HardwareSerial Serial2(PA3, PA2);

/**
 * Blink the LED `count` times with given on/off timing (ms). Blocking.
 */
static void ledBlink(uint8_t count, uint16_t onMs, uint16_t offMs)
{
    for (uint8_t i = 0; i < count; i++)
    {
        digitalWrite(LED_PIN, LOW);   // ON  (active LOW on Black Pill PC13)
        delay(onMs);
        digitalWrite(LED_PIN, HIGH);  // OFF
        if (i < count - 1) delay(offMs);
    }
}

// =============================================================================
// HELPERS
// =============================================================================

static void transmitCombinedSpectrum()
{
    for (uint16_t i = 0; i < GAMMA_ADC_BINS; i++)
    {
        Serial6.print(gamma_combined[i]);
        Serial6.print(';');
    }
    Serial6.print('\n');
    Serial6.flush();
}

// Called every ~200ms during GAMMA_Init() health check waits.
// Rapid alternating blink so you can see init is alive and waiting.
static void initTick()
{
    static bool state = false;
    state = !state;
    digitalWrite(LED_PIN, state ? LOW : HIGH);  // active LOW
}

// =============================================================================
// SETUP
// =============================================================================

void setup()
{
    // ---- LED init — turn on solid to show we have power and setup is running ----
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);  // ON solid during init (active LOW on Black Pill PC13)

    // ---- Downlink serial (Serial6: TX=PC6, RX=PC7) ----
    Serial6.begin(DOWNLINK_BAUD);

    // ---- Gamma spectrometer init ----
    // Pass initTick so the LED blinks during the health check wait.
    GammaInitStatus gammaStatus = GAMMA_Init(initTick);

    if (gammaStatus == GAMMA_BOTH_MISSING)
    {
        while (true) { ledBlink(1, 100, 100); }  // rapid blink — fatal halt
    }

    // ---- Signal init outcome on LED ----
    digitalWrite(LED_PIN, HIGH);  // OFF before blinking (active LOW)
    delay(300);

    if (gamma_a_online && gamma_b_online)
        ledBlink(2, 100, 100);    // 2 fast blinks — both online
    else
        ledBlink(1, 500, 0);      // 1 slow blink  — partial init

    delay(300);
    ledBlink(3, 300, 200);        // 3 slow blinks — setup() complete
}

// =============================================================================
// LOOP
// =============================================================================

void loop()
{
    // ---- Poll gamma spectrometers ----
    GAMMA_Poll();

    // ---- Per-detector receive indicator (non-blocking LED bursts) ----
    static uint8_t       ledBursts = 0;
    static uint8_t       ledStep   = 0;
    static unsigned long ledStepMs = 0;

    if (gamma_new_A)
    {
        // 1 short pulse — histogram received from A
        ledBursts = 1; ledStep = 0; ledStepMs = millis();
        digitalWrite(LED_PIN, LOW);  // ON
    }
    if (gamma_new_B)
    {
        // 2 short pulses — histogram received from B
        ledBursts = 2; ledStep = 0; ledStepMs = millis();
        digitalWrite(LED_PIN, LOW);  // ON
    }

    // Drive LED burst state machine — 30ms on, 60ms off per pulse
    if (ledBursts > 0 && (millis() - ledStepMs >= (ledStep % 2 == 0 ? 30 : 60)))
    {
        ledStep++;
        ledStepMs = millis();
        if (ledStep % 2 == 0)
        {
            ledBursts--;
            digitalWrite(LED_PIN, ledBursts > 0 ? LOW : HIGH);  // LOW=ON, HIGH=OFF
        }
        else
        {
            digitalWrite(LED_PIN, HIGH);  // OFF between pulses
        }
    }

    // ---- Heartbeat: 1 short pulse every second (non-blocking) ----
    static unsigned long lastHeartbeat = 0;
    static bool          heartbeatOn   = false;
    const  unsigned long now           = millis();

    if (!heartbeatOn && (now - lastHeartbeat >= 1000))
    {
        lastHeartbeat = now;
        heartbeatOn   = true;
        if (ledBursts == 0) digitalWrite(LED_PIN, LOW);   // ON
    }
    if (heartbeatOn && (now - lastHeartbeat >= 20))
    {
        heartbeatOn = false;
        if (ledBursts == 0) digitalWrite(LED_PIN, HIGH);  // OFF
    }

    // ---- Transmit combined spectrum every 6 seconds ----
    static unsigned long lastTransmit = 0;

    if (now - lastTransmit >= TRANSMIT_INTERVAL_MS)
    {
        lastTransmit = now;
        transmitCombinedSpectrum();

        // 2 quick pulses to confirm TX fired
        ledBursts = 2; ledStep = 0; ledStepMs = millis();
        digitalWrite(LED_PIN, LOW);  // ON
    }
}