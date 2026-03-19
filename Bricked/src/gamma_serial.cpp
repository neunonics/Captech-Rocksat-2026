#include "gamma_serial.h"

// =============================================================================
// HARDWARE SERIAL INSTANCES — STM32F411CEU6 (Black Pill)
//
// The STM32 Arduino core does not instantiate Serial1/2/6 automatically.
// We declare them here with explicit STM32 pin names. These definitions are
// visible to the linker for the whole sketch via the extern declarations in
// gamma_serial.h.
//
// Pin mapping:
//   Serial1 — USART1: TX=PA9,  RX=PA10  — Spectrometer A
//   Serial2 — USART2: TX=PA2,  RX=PA3   — Spectrometer B
//   Serial6 — USART6: TX=PC6,  RX=PC7   — Combined spectrum downlink
// =============================================================================
// HardwareSerial Serial1(PA_10, PA_9);   // RX, TX
// HardwareSerial Serial2(PA_3,  PA_2);   // RX, TX
// HardwareSerial Serial6(PC_7,  PC_6);   // RX, TX

// =============================================================================
// GLOBAL VARIABLE DEFINITIONS
// =============================================================================
bool     gamma_ready         = false;
bool     gamma_a_online      = false;
bool     gamma_b_online      = false;
uint32_t gamma_lines_saved_A = 0;
uint32_t gamma_lines_saved_B = 0;
uint16_t gamma_combined[GAMMA_ADC_BINS] = {0};
uint32_t gamma_combined_count = 0;

// Pulses true for one Poll() cycle when a fresh histogram arrives from each
// detector. Read these in main.cpp to flash an LED per detector.
bool gamma_new_A = false;
bool gamma_new_B = false;

// Number of bins parsed from the most recent complete line per detector.
// Should equal the OGD's ADC_BINS (4096) if the full line is being received.
// Will equal GAMMA_ADC_BINS if we hit our storage limit first.
uint16_t gamma_last_bins_A = 0;
uint16_t gamma_last_bins_B = 0;

// =============================================================================
// INTERNAL STATE — not exposed in the header
// =============================================================================

// SD file handles — commented out, SD logging disabled
// static File sdFileA;
// static File sdFileB;

// Flags used internally to track which detectors have contributed a fresh
// histogram to gamma_combined since the last combination.
static bool pendingA = false;
static bool pendingB = false;

// Per-detector on-the-fly parse state.
// Instead of buffering the entire line we accumulate one numeric token at a
// time in a small char buffer, parse it when we hit ';', and write directly
// into gamma_combined. This keeps RAM usage constant regardless of line length.
#define TOKEN_BUF_SIZE 11  // max 10 digits for uint32 + null terminator

struct PortState {
    char          token[TOKEN_BUF_SIZE];  // current numeric token being built
    uint8_t       tokenLen;               // bytes written into token[]
    uint16_t      binIndex;               // which bin we are writing next (capped at GAMMA_ADC_BINS)
    uint16_t      totalBins;             // total bins seen in this line (uncapped)
    unsigned long lastByteMs;             // timestamp of last received byte
};

static PortState stateA = {"", 0, 0, 0, 0};
static PortState stateB = {"", 0, 0, 0, 0};

// =============================================================================
// INTERNAL HELPERS
// =============================================================================

/**
 * Flush the TX buffer of a serial port and clear its RX buffer.
 * Used to ensure no stale bytes linger before we send a command.
 */
static void flushSerial(HardwareSerial &port)
{
    port.flush();                        // Wait for TX to complete
    while (port.available()) port.read(); // Discard any buffered RX bytes
}

/**
 * Send a shell command to a spectrometer.
 * SimpleShell_Enhanced on the OGD side expects a newline-terminated string.
 */
static void sendCommand(HardwareSerial &port, const char *cmd)
{
    flushSerial(port);
    port.print(cmd);
    port.flush();
}

/**
 * Wait up to timeoutMs for a response line containing the given substring.
 * Calls tickFn() every 200ms while waiting so the caller can blink an LED.
 * tickFn may be nullptr if no tick is needed.
 * Returns true if the expected string was found within the timeout.
 */
static bool waitForResponse(HardwareSerial &port,
                             const char    *expected,
                             uint32_t       timeoutMs,
                             void         (*tickFn)() = nullptr)
{
    // Fixed 64-byte rolling buffer — enough to detect the health string
    // without any heap allocation
    const uint8_t BUF_SIZE = 64;
    char     buf[BUF_SIZE];
    uint8_t  head    = 0;   // next write position (circular)
    uint8_t  filled  = 0;   // how many bytes are in the buffer
    uint8_t  expLen  = strlen(expected);

    const unsigned long deadline = millis() + timeoutMs;
    unsigned long lastTick = 0;

    while (millis() < deadline)
    {
        if (tickFn && (millis() - lastTick >= 200))
        {
            lastTick = millis();
            tickFn();
        }

        while (port.available())
        {
            buf[head] = (char)port.read();
            head = (head + 1) % BUF_SIZE;
            if (filled < BUF_SIZE) filled++;

            // Check if expected string is present in the circular buffer.
            // Reconstruct a linear copy for strstr.
            if (filled >= expLen)
            {
                char linear[BUF_SIZE + 1];
                uint8_t start = (head + BUF_SIZE - filled) % BUF_SIZE;
                for (uint8_t i = 0; i < filled; i++)
                {
                    linear[i] = buf[(start + i) % BUF_SIZE];
                }
                linear[filled] = '\0';
                if (strstr(linear, expected) != nullptr)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

/**
 * Drain available bytes from `port`, parsing semicolon-delimited tokens
 * on-the-fly directly into gamma_combined. No heap allocation required.
 *
 * Protocol: each OGD line is "val;val;val;...val;\n"
 *   - digits accumulate in ps.token[]
 *   - on ';' the token is parsed and added to gamma_combined[ps.binIndex++]
 *   - on '\n' the line is complete — set pendingFlag, newFlag, increment count
 *   - bins beyond GAMMA_ADC_BINS are ignored (index stops advancing)
 *   - if no byte arrives for GAMMA_MAX_LINE_MS the parse state is reset
 */
static void drainPort(HardwareSerial &port,
                      PortState      &ps,
                      bool           &pendingFlag,
                      bool           &newFlag,
                      uint16_t       &lastBins,
                      uint32_t       &savedCount,
                      bool            sniff = false)
{
    newFlag = false;

    while (port.available())
    {
        char c = (char)port.read();
        ps.lastByteMs = millis();

#if GAMMA_RAW_SNIFF
        if (sniff)
        {
            // Print every byte as-is so we can see the exact OGD output format.
            // Non-printable characters shown as hex e.g. <0A> for newline.
            if (c >= 0x20 && c < 0x7F)
                Serial.print(c);
            else
            {
                Serial.print('<');
                if (c < 0x10) Serial.print('0');
                Serial.print((uint8_t)c, HEX);
                Serial.print('>');
            }
        }
#endif

        if (c == ';')
        {
            // End of one token — parse and accumulate if within range
            if (ps.tokenLen > 0)
            {
                if (ps.binIndex < GAMMA_ADC_BINS)
                {
                    ps.token[ps.tokenLen] = '\0';
                    uint32_t val = (uint32_t)atol(ps.token);
                    // Clamp to uint16_t max to prevent overflow
                    uint32_t sum = gamma_combined[ps.binIndex] + val;
                    gamma_combined[ps.binIndex] = (uint16_t)(sum > 65535 ? 65535 : sum);
                    ps.binIndex++;
                }
                ps.totalBins++;  // count every bin seen, even beyond GAMMA_ADC_BINS
            }
            ps.tokenLen = 0;
        }
        else if (c == '\n')
        {
            // End of line — save bin count and signal completion
            lastBins     = ps.totalBins;
            ps.tokenLen  = 0;
            ps.binIndex  = 0;
            ps.totalBins = 0;
            pendingFlag  = true;
            newFlag      = true;
            savedCount++;
        }
        else if (c != '\r' && ps.tokenLen < TOKEN_BUF_SIZE - 1)
        {
            ps.token[ps.tokenLen++] = c;
        }
    }

    // Time-out guard: reset parse state if no byte for too long
    if ((ps.tokenLen > 0 || ps.binIndex > 0) &&
        (millis() - ps.lastByteMs) > GAMMA_MAX_LINE_MS)
    {
        ps.tokenLen  = 0;
        ps.binIndex  = 0;
        ps.totalBins = 0;
    }
}

// =============================================================================
// PUBLIC API
// =============================================================================

GammaInitStatus GAMMA_Init(void (*tickFn)())
{
    gamma_ready    = false;
    gamma_a_online = false;
    gamma_b_online = false;

    // ---- 1. Start serial ports ----
    GAMMA_SERIAL_A.begin(GAMMA_BAUD_RATE);
    GAMMA_SERIAL_B.begin(GAMMA_BAUD_RATE);
    delay(100);

#if GAMMA_SKIP_HEALTH_CHECK
    // Health check and reboot bypassed — assume both detectors are streaming.
    // Flip to 0 in gamma_serial.h once comms are confirmed working.
    gamma_a_online = true;
    gamma_b_online = true;
#else
    // ---- 2. Health check — send "read info\n" to each, look for the banner ----
    sendCommand(GAMMA_SERIAL_A, GAMMA_CMD_INFO);
    gamma_a_online = waitForResponse(GAMMA_SERIAL_A, GAMMA_HEALTH_STRING, GAMMA_INIT_TIMEOUT_MS, tickFn);

    sendCommand(GAMMA_SERIAL_B, GAMMA_CMD_INFO);
    gamma_b_online = waitForResponse(GAMMA_SERIAL_B, GAMMA_HEALTH_STRING, GAMMA_INIT_TIMEOUT_MS, tickFn);

    if (!gamma_a_online && !gamma_b_online)
    {
        return GAMMA_BOTH_MISSING;
    }

    // ---- 3. Reboot all online detectors as simultaneously as possible ----
    if (gamma_a_online) flushSerial(GAMMA_SERIAL_A);
    if (gamma_b_online) flushSerial(GAMMA_SERIAL_B);

    if (gamma_a_online) GAMMA_SERIAL_A.print(GAMMA_CMD_REBOOT);
    if (gamma_b_online) GAMMA_SERIAL_B.print(GAMMA_CMD_REBOOT);
    if (gamma_a_online) GAMMA_SERIAL_A.flush();
    if (gamma_b_online) GAMMA_SERIAL_B.flush();

    // ---- 4. Wait for online detectors to reboot and begin streaming ----
    delay(GAMMA_REBOOT_SETTLE_MS);

    if (gamma_a_online) { while (GAMMA_SERIAL_A.available()) GAMMA_SERIAL_A.read(); }
    if (gamma_b_online) { while (GAMMA_SERIAL_B.available()) GAMMA_SERIAL_B.read(); }
#endif

    // ---- 5. SD card initialisation — currently disabled ----
    // Uncomment this block to re-enable SD logging:
    //
    // if (!SD.begin(SD_CS_PIN))
    // {
    //     return GAMMA_SD_FAIL;
    // }
    // sdFileA = SD.open(SD_FILE_A, FILE_WRITE);
    // sdFileB = SD.open(SD_FILE_B, FILE_WRITE);
    // if (!sdFileA || !sdFileB)
    // {
    //     return GAMMA_SD_FAIL;
    // }

    // Clear parse state in case Init() is called more than once
    memset(&stateA, 0, sizeof(stateA));
    memset(&stateB, 0, sizeof(stateB));
    pendingA     = false;
    pendingB     = false;
    gamma_new_A  = false;
    gamma_new_B  = false;
    memset(gamma_combined, 0, sizeof(gamma_combined));
    gamma_lines_saved_A  = 0;
    gamma_lines_saved_B  = 0;
    gamma_combined_count = 0;

    gamma_ready = true;
    return GAMMA_OK;
}

// -----------------------------------------------------------------------------

void GAMMA_Poll()
{
    if (!gamma_ready) return;

    if (gamma_a_online)
        drainPort(GAMMA_SERIAL_A, stateA, pendingA, gamma_new_A, gamma_last_bins_A, gamma_lines_saved_A, true);
    if (gamma_b_online)
        drainPort(GAMMA_SERIAL_B, stateB, pendingB, gamma_new_B, gamma_last_bins_B, gamma_lines_saved_B, false);

    const bool aReady = !gamma_a_online || pendingA;
    const bool bReady = !gamma_b_online || pendingB;

    if (aReady && bReady)
    {
        gamma_combined_count++;
        pendingA = false;
        pendingB = false;
    }
}

// -----------------------------------------------------------------------------

void GAMMA_Close()
{
    // SD logging disabled — uncomment to re-enable:
    // if (sdFileA) { sdFileA.flush(); sdFileA.close(); }
    // if (sdFileB) { sdFileB.flush(); sdFileB.close(); }
    gamma_ready = false;
}

// -----------------------------------------------------------------------------

void GAMMA_ResetCombined()
{
    memset(gamma_combined, 0, sizeof(gamma_combined));
    memset(&stateA, 0, sizeof(stateA));
    memset(&stateB, 0, sizeof(stateB));
    gamma_combined_count = 0;
    pendingA    = false;
    pendingB    = false;
    gamma_new_A = false;
    gamma_new_B = false;
}