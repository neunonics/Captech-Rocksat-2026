#ifndef GAMMA_SERIAL_H
#define GAMMA_SERIAL_H

#include <Arduino.h>

// =============================================================================
// SERIAL PORT INSTANCES — defined in gamma_serial.cpp
// Declared extern here so main.ino and any other TU can reference them.
// =============================================================================
extern HardwareSerial Serial1;   // USART1 — TX=PA_9,  RX=PA_10 — Spectrometer A
extern HardwareSerial Serial2;   // USART2 — TX=PA_2,  RX=PA_3  — Spectrometer B
extern HardwareSerial Serial6;   // USART6 — TX=PC_6,  RX=PC_7  — Downlink
// #include <SD.h>  // SD logging currently disabled

// =============================================================================
// CONFIGURATION
// =============================================================================

// Baud rate must match the OGD firmware (shell.begin / shell1.begin in ogd_pico.ino)
#define GAMMA_BAUD_RATE       115200

// =============================================================================
// SERIAL PORT CONFIGURATION — STM32F411CEU6 (Black Pill)
//
// The STM32 Arduino core pre-defines Serial1/2/6 once ENABLE_HWSERIAL* is set.
//
//   Serial  (USB CDC)  — debug monitor (no physical UART pins needed)
//   Serial1 (USART1)   — TX=PA9,  RX=PA10 — Spectrometer A
//   Serial2 (USART2)   — TX=PA2,  RX=PA3  — Spectrometer B
//   Serial6 (USART6)   — TX=PC6,  RX=PC7  — Combined spectrum downlink
//
// NOTE: The OGD spectrometer is a 3.3V device. The Black Pill also runs at
// 3.3V so no level shifter is required.
// =============================================================================
#define GAMMA_SERIAL_A        Serial1   // Spectrometer A: TX=PA9,  RX=PA10
#define GAMMA_SERIAL_B        Serial2   // Spectrometer B: TX=PA2,  RX=PA3

// SD chip-select pin — adjust to match your hardware
// #define SD_CS_PIN             10        // TODO: verify against schematic

// Log filenames written to SD card (one file per spectrometer)
// #define SD_FILE_A             "specA.txt"
// #define SD_FILE_B             "specB.txt"

// Set to 1 to print raw bytes received from Spectrometer A directly to Serial.
// Use this to inspect the exact format the OGD is sending before parsing.
// Set back to 0 for normal operation.
#define GAMMA_RAW_SNIFF          0
#define GAMMA_SKIP_HEALTH_CHECK  0

// Serial6 (TX=PC6) outputs the raw histogram only — no status messages.
// Format: semicolon-delimited bin values, newline-terminated, 4096 bins per line.

// How long to wait for a health-check response from each spectrometer (ms)
#define GAMMA_INIT_TIMEOUT_MS 5000

// How long to wait between sending the reboot command to each spectrometer
// and polling for new data (ms) — give them time to come back up
#define GAMMA_REBOOT_SETTLE_MS 3000

// String that must appear in a health-check response to consider the
// spectrometer alive. Matches the "Open Gamma Detector" banner from read info.
#define GAMMA_HEALTH_STRING   "Open Gamma Detector"

// Commands sent to the OGD shell (try \r\n if \n alone doesn't get a response)
#define GAMMA_CMD_INFO        "read info\r\n"
#define GAMMA_CMD_REBOOT      "reboot\r\n"

// Maximum length of a single incoming data line (semicolon-delimited histogram)
// OGD default ADC_BINS = 4096 channels, each up to ~10 chars + ';' = ~57 kB worst case.
// We use a heap-allocated String so the stack is not blown; this constant is only
// used for the read-timeout guard.
#define GAMMA_MAX_LINE_MS     5000      // Abandon a partial line after this many ms of silence

// Number of ADC channels to store. The OGD sends 4096 bins.
// The STM32F411 has 128 kB RAM so storing all 4096 bins is fine:
// 4096 × 2 bytes (uint16_t) = 8 kB for gamma_combined.
#define GAMMA_ADC_BINS        4096

// =============================================================================
// STATUS TYPE
// Returned by GAMMA_Init() so main.cpp can react to partial failures.
// =============================================================================
enum GammaInitStatus {
    GAMMA_OK          = 0,   // Both spectrometers found and rebooted
    GAMMA_A_MISSING   = 1,   // Spectrometer A did not respond
    GAMMA_B_MISSING   = 2,   // Spectrometer B did not respond
    GAMMA_BOTH_MISSING= 3,   // Neither spectrometer responded
    // GAMMA_SD_FAIL  = 4    // SD card could not be initialised (SD disabled)
};

// =============================================================================
// GLOBAL STATE — read these in main.cpp if needed
// =============================================================================

// Set to true once GAMMA_Init() completes — true even if only one detector is online
extern bool gamma_ready;

// Per-detector online flags — check these in main.cpp after GAMMA_Init()
// to know which detectors are active. Set by GAMMA_Init(), never changed after.
extern bool gamma_a_online;
extern bool gamma_b_online;

// Running count of lines received from each spectrometer
// (SD logging disabled — uncomment SD sections to re-enable saving)
extern uint32_t gamma_lines_saved_A;
extern uint32_t gamma_lines_saved_B;

// Combined spectrum — bin-by-bin sum of the latest histogram from A and/or B.
// uint16_t (max 65535 counts per bin) is sufficient for typical flight durations
// at expected count rates. Switch to uint32_t if running long ground tests where
// bins could exceed 65535.
extern uint16_t gamma_combined[GAMMA_ADC_BINS];

// Incremented each time gamma_combined is updated. Use this in main.cpp to
// detect when a new combined spectrum is available without polling the array directly.
extern uint32_t gamma_combined_count;

// Pulsed true for one Poll() cycle when a fresh histogram arrives from each
// detector. Use these in main.cpp to flash an indicator LED per detector.
extern bool gamma_new_A;
extern bool gamma_new_B;

// Total bins parsed from the most recent complete line per detector.
// Should equal the OGD's ADC_BINS (4096) if the full line is being received.
// Compare against GAMMA_ADC_BINS to see how many bins are being stored.
extern uint16_t gamma_last_bins_A;
extern uint16_t gamma_last_bins_B;

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================

/**
 * @brief  Initialise spectrometers. Continues with partial success if only
 *         one detector responds — check gamma_a_online / gamma_b_online after
 *         calling to know which are active.
 *
 *         tickFn is an optional callback fired every ~200ms during health check
 *         waits — pass a function that toggles an LED so you can see init is
 *         still running. Pass nullptr if not needed.
 *
 * @return GAMMA_OK           — at least one detector online, gamma_ready true.
 *         GAMMA_BOTH_MISSING — no detectors responded, gamma_ready false.
 */
GammaInitStatus GAMMA_Init(void (*tickFn)() = nullptr);

/**
 * @brief  Poll both serial ports for new complete lines.
 *
 *         Must be called as frequently as possible from loop() — each call
 *         drains whatever bytes are currently available in both RX buffers
 *         and assembles complete '\n'-terminated lines. Incomplete lines are
 *         held in internal buffers until the next call.
 *         (SD logging disabled — lines are counted but not written to disk.)
 *
 *         Does nothing (returns immediately) if gamma_ready is false.
 */
void GAMMA_Poll();

/**
 * @brief  Close SD log files. Currently a no-op (SD disabled).
 *         Uncomment SD sections to re-enable.
 */
void GAMMA_Close();

/**
 * @brief  Zero out the combined spectrum and reset the combined count.
 *         Call this whenever you want to start a fresh accumulation.
 */
void GAMMA_ResetCombined();

#endif // GAMMA_SERIAL_H