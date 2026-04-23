/**
 * inst.h
 * Teensy 4.1 – Dual Gamma Spectrometer Interface
 *
 * Spectrometer 1 → Serial6  (UART6)
 * Spectrometer 2 → Serial3  (UART3)
 * Output          → Serial1  (change OUTPUT_SERIAL to suit)
 *
 * Requires: SD.h  (Teensy built-in, uses onboard SDIO)
 */

#ifndef INST_H
#define INST_H

#include <Arduino.h>
#include <SD.h>
#include <math.h>

/* ─────────────────────────────────────────────────────────────
 * Hardware / protocol configuration
 * ───────────────────────────────────────────────────────────── */

#define SPEC1_SERIAL            Serial6     /* Spectrometer 1 – UART6       */
#define SPEC2_SERIAL            Serial3     /* Spectrometer 2 – UART3       */
#define OUTPUT_SERIAL           Serial8     /* Combined histogram TX target  */

#define SPEC_BAUD               115200
#define OUTPUT_BAUD             115200

/** Number of energy bins per spectrometer */
#define HISTOGRAM_BINS          4096

/** Max ms to wait for a single '\n'-terminated line */
#define UART_LINE_TIMEOUT_MS    500

/** Delay between connection-probe retries, ms */
#define CONNECT_RETRY_DELAY_MS  1000

/** Consecutive bad-quality reads before both units are rebooted */
#define FAULT_STRIKE_LIMIT      3

/** Minimum acceptable total counts across a valid histogram */
#define MIN_TOTAL_COUNTS        10UL

/** Fraction of empty bins above which the histogram is flagged sparse */
#define MAX_ZERO_BIN_FRACTION   0.99f

/** Teensy 4.1 built-in SD card (SDIO) */
#define SD_CS_PIN               BUILTIN_SDCARD

#define SD_LOG_FILE_1           "spec1.csv"
#define SD_LOG_FILE_2           "spec2.csv"
#define SD_LOG_FILE_COMBINED    "combined.csv"

#define INFERENCE_BUF_LEN 256
extern char inference[INFERENCE_BUF_LEN];


/* ─────────────────────────────────────────────────────────────
 * Types
 * ───────────────────────────────────────────────────────────── */

typedef enum : int8_t {
    SPEC_OK          =  0,
    SPEC_ERR_UART    = -1,
    SPEC_ERR_TIMEOUT = -2,
    SPEC_ERR_PARSE   = -3,
    SPEC_ERR_SD      = -4,
} SpecStatus;

typedef enum : uint8_t {
    FAULT_NONE        = 0,
    FAULT_LOW_COUNTS  = (1 << 0), /* total counts below MIN_TOTAL_COUNTS    */
    FAULT_SPARSE_BINS = (1 << 1), /* too many empty bins                    */
    FAULT_NAN_RAW     = (1 << 2), /* NaN / unparse-able token in raw stream */
    FAULT_UART        = (1 << 3), /* UART read/write error                  */
    FAULT_OVERFLOW    = (1 << 4), /* bin value implausibly large            */
} FaultFlags;

inline FaultFlags operator|(FaultFlags a, FaultFlags b)
{
    return static_cast<FaultFlags>(
        static_cast<uint8_t>(a) | static_cast<uint8_t>(b)
    );
}

inline FaultFlags& operator|=(FaultFlags& a, FaultFlags b)
{
    a = a | b;
    return a;
}

inline FaultFlags operator&(FaultFlags a, FaultFlags b)
{
    return static_cast<FaultFlags>(
        static_cast<uint8_t>(a) & static_cast<uint8_t>(b)
    );
}

inline bool anyFault(FaultFlags f)
{
    return static_cast<uint8_t>(f) != 0;
}

struct Histogram {
    uint32_t   bins[HISTOGRAM_BINS];
    uint32_t   total_counts;
    bool       valid;
    FaultFlags faults;
    uint8_t    strike_count;  /* consecutive fault counter */
};

struct CombinedHistogram {
    uint32_t    bins[HISTOGRAM_BINS]; /* float, NaN-scrubbed, ready to TX */
    uint32_t total_counts;
};

/* ─────────────────────────────────────────────────────────────
 * Public API
 * ───────────────────────────────────────────────────────────── */

/** Initialise SD card – call once from setup(). Halts on failure. */
void       SPEC_InitSD(void);

/** Block until both spectrometers respond to ID probe. Retries forever. */
void       SPEC_InitUntilConnected(void);

/** Send reboot command simultaneously to both spectrometers to sync output. */
void       SPEC_SyncReboot(void);

/** Read full histogram from spectrometer 1 (Serial6). */
SpecStatus SPEC_ReadHistogram1(Histogram &dst);

/** Read full histogram from spectrometer 2 (Serial3). */
SpecStatus SPEC_ReadHistogram2(Histogram &dst);

/** Append a raw histogram to a CSV file on the SD card. */
SpecStatus SPEC_LogHistogramSD(const Histogram &hist, const char *filename);

/** Element-wise float sum: hist1 + hist2 → dst. */
void       SPEC_CombineHistograms(const Histogram   &hist1,
                                  const Histogram   &hist2,
                                  CombinedHistogram &dst);

/** Replace any NaN / Inf in the combined histogram with 0. */
void       SPEC_ScrubNaN(CombinedHistogram &hist);

/** Write combined histogram to SD. */
SpecStatus SPEC_LogCombinedSD(const CombinedHistogram &hist);

/** Stream combined histogram over OUTPUT_SERIAL as ASCII. */
void       SPEC_TransmitCombined(const CombinedHistogram &hist);

/** Check histogram quality; updates faults + strike_count. Returns fault mask. */
FaultFlags SPEC_CheckQuality(Histogram &hist);

/** If either spectrometer has exceeded FAULT_STRIKE_LIMIT, reboot both. */
void       SPEC_HandleFaults(Histogram &hist1, Histogram &hist2);

/** Main acquisition loop – call from loop() or a FreeRTOS task. */
void       SPEC_AcquisitionLoop(void);

/** Poll OUTPUT_SERIAL for a new line of inference from ORIN. */
String       ORIN_Poll(void);

#endif /* INST_H */