/**
 * inst.cpp
 * Teensy 4.1 – Dual Gamma Spectrometer Interface
 *
 * Stream format (continuous, no request needed):
 *   "<uint32>;<uint32>;...;<uint32>\r\n"   (HISTOGRAM_BINS values per line)
 *
 * Spectrometer 1 → SPEC1_SERIAL  (UART6, Teensy 4.1 pins 44/45)
 * Spectrometer 2 → SPEC2_SERIAL  (UART3, Teensy 4.1 pins 14/15)
 */

#include "inst.h"

/* ─────────────────────────────────────────────────────────────
 * RX buffers – must be large enough to hold one full histogram
 * line while the other port is being drained.
 * HISTOGRAM_BINS × 7 bytes (6 digits + delimiter) rounded up.
 * ───────────────────────────────────────────────────────────── */
static uint8_t rxBuf1[HISTOGRAM_BINS * 7];
static uint8_t rxBuf2[HISTOGRAM_BINS * 7];
char inference[INFERENCE_BUF_LEN];

/* ─────────────────────────────────────────────────────────────
 * Internal helpers
 * ───────────────────────────────────────────────────────────── */

/* readLine is kept for probe / reboot response parsing. */
static SpecStatus readLine(HardwareSerial &port, char *buf, size_t bufLen)
{
    size_t   idx      = 0;
    uint32_t deadline = millis() + UART_LINE_TIMEOUT_MS;

    while (millis() < deadline)
    {
        if (!port.available()) { yield(); continue; }

        int c = port.read();
        if (c < 0) continue;

        if (c == '\n') { buf[idx] = '\0'; return SPEC_OK; }
        if (c == '\r') continue;

        if (idx >= bufLen - 1) { buf[idx] = '\0'; return SPEC_ERR_UART; }
        buf[idx++] = (char)c;
    }

    buf[idx] = '\0';
    return SPEC_ERR_TIMEOUT;
}

/*
 * Parses one complete semicolon-delimited histogram line from the
 * continuous stream.  Attempts twice so a partial line (joined
 * mid-stream on first call) is discarded in favour of the next
 * clean frame.  Deadline slides on every received byte so a large
 * bin count does not cause false timeouts.
 */
static SpecStatus readHistogramFrom(HardwareSerial &port, Histogram &dst)
{
    memset(&dst, 0, sizeof(dst));
    dst.valid  = false;
    dst.faults = FAULT_NONE;

    for (int attempt = 0; attempt < 2; ++attempt)
    {
        uint32_t deadline    = millis() + UART_LINE_TIMEOUT_MS;
        int      bin         = 0;
        long     acc         = 0;
        bool     hasDigit    = false;
        bool     tokenActive = false;   /* any char seen for current token */

        memset(dst.bins, 0, sizeof(dst.bins));
        dst.total_counts = 0;
        dst.faults       = FAULT_NONE;

        while (true)
        {
            if (millis() >= deadline) {
                dst.faults |= FAULT_UART;
                return SPEC_ERR_TIMEOUT;
            }
            if (!port.available()) { yield(); continue; }

            int c = port.read();
            if (c < 0) continue;
            deadline = millis() + UART_LINE_TIMEOUT_MS;

            if (c == '\r') continue;

            if (c >= '0' && c <= '9') {
                acc         = acc * 10 + (c - '0');
                hasDigit    = true;
                tokenActive = true;
            } else if (c == ';' || c == '\n') {
                if (tokenActive && bin < HISTOGRAM_BINS) {
                    if (!hasDigit) {
                        /* non-numeric token (nan/inf/etc.) */
                        dst.bins[bin] = 0;
                        dst.faults   |= FAULT_NAN_RAW;
                    } else {
                        if (acc > 0x00FFFFFFL) dst.faults |= FAULT_OVERFLOW;
                        dst.bins[bin]       = (uint32_t)acc;
                        dst.total_counts   += (uint32_t)acc;
                    }
                    bin++;
                }
                acc         = 0;
                hasDigit    = false;
                tokenActive = false;

                if (c == '\n') break;
            } else {
                tokenActive = true;   /* letter in nan/inf/etc. */
            }
        }

        if (bin == HISTOGRAM_BINS) {
            dst.valid = true;
            return SPEC_OK;
        }
        /* partial line — discard and read the next complete one */
    }

    dst.faults |= FAULT_UART;
    return SPEC_ERR_PARSE;
}

/* ─────────────────────────────────────────────────────────────
 * Public API
 * ───────────────────────────────────────────────────────────── */

void SPEC_InitSD(void)
{
    Serial.println("[SPEC] Initialising SD card...");

    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("[SPEC] ERROR: SD card init failed! Halting.");
        while (true) { delay(500); }
    }

    Serial.println("[SPEC] SD card ready.");
}

void SPEC_InitUntilConnected(void)
{
    SPEC1_SERIAL.begin(SPEC_BAUD);
    SPEC2_SERIAL.begin(SPEC_BAUD);

    /* Enlarge RX buffers so one port can accumulate a full histogram
     * line while the other is being drained by readHistogramFrom. */
    SPEC1_SERIAL.addMemoryForRead(rxBuf1, sizeof(rxBuf1));
    SPEC2_SERIAL.addMemoryForRead(rxBuf2, sizeof(rxBuf2));

    Serial.println("[SPEC] Waiting for spectrometers...");

    bool s1 = false, s2 = false;

    while (!s1 || !s2)
    {
        if (!s1) {
            while (!SPEC1_SERIAL) { delay(10); }
            s1 = true;
            Serial.println("[SPEC] Spectrometer 1 connected (SPEC1_SERIAL).");
        }
        if (!s2) {
            while (!SPEC2_SERIAL) { delay(10); }
            s2 = true;
            Serial.println("[SPEC] Spectrometer 2 connected (SPEC2_SERIAL).");
        }

        if (!s1 || !s2) delay(CONNECT_RETRY_DELAY_MS);
    }

    Serial.println("[SPEC] Both spectrometers ready.");
}

void SPEC_SyncReboot(void)
{
    SPEC1_SERIAL.print("reboot\r\n");
    SPEC2_SERIAL.print("reboot\r\n");

    Serial.println("[SPEC] Reboot commands sent. Waiting for boot...");
    delay(500);
}

SpecStatus SPEC_ReadHistogram1(Histogram &dst)
{
    return readHistogramFrom(SPEC1_SERIAL, dst);
}

SpecStatus SPEC_ReadHistogram2(Histogram &dst)
{
    return readHistogramFrom(SPEC2_SERIAL, dst);
}

SpecStatus SPEC_LogHistogramSD(const Histogram &hist, const char *filename)
{
    File f = SD.open(filename, FILE_WRITE);
    if (!f) {
        Serial.printf("[SPEC] ERROR: Cannot open %s for writing.\n", filename);
        return SPEC_ERR_SD;
    }

    f.println("bin_index,counts");
    for (int i = 0; i < HISTOGRAM_BINS; ++i) {
        f.print(i);
        f.print(',');
        f.println(hist.bins[i]);
    }

    f.close();
    return SPEC_OK;
}

void SPEC_CombineHistograms(const Histogram   &hist1,
                            const Histogram   &hist2,
                            CombinedHistogram &dst)
{
    dst.total_counts = 0;

    for (int i = 0; i < HISTOGRAM_BINS; ++i)
    {
        float combined = (float)hist1.bins[i] + (float)hist2.bins[i];
        if (!isfinite(combined)) combined = 0.0f;

        dst.bins[i]       = combined;
        dst.total_counts += (uint32_t)combined;
    }

    dst.bins[0]       = 0.0f;
    dst.total_counts -= (uint32_t)(hist1.bins[0] + hist2.bins[0]);
}

void SPEC_ScrubNaN(CombinedHistogram &hist)
{
    hist.total_counts = 0;

    for (int i = 0; i < HISTOGRAM_BINS; ++i)
    {
        if (isnan(hist.bins[i]) || isinf(hist.bins[i]))
            hist.bins[i] = 0;

        hist.total_counts += (uint32_t)hist.bins[i];
    }
}

SpecStatus SPEC_LogCombinedSD(const CombinedHistogram &hist)
{
    File f = SD.open(SD_LOG_FILE_COMBINED, FILE_WRITE);
    if (!f) {
        Serial.println("[SPEC] ERROR: Cannot open combined log file.");
        return SPEC_ERR_SD;
    }

    f.println("bin_index,counts");
    for (int i = 0; i < HISTOGRAM_BINS; ++i) {
        f.print(i);
        f.print(',');
        f.println((uint32_t)hist.bins[i]);
    }

    f.close();
    return SPEC_OK;
}

void SPEC_TransmitCombined(const CombinedHistogram &hist)
{
    for (int i = 0; i < HISTOGRAM_BINS; ++i) {
        //if (i > 0) OUTPUT_SERIAL.print(';');
        OUTPUT_SERIAL.print(hist.bins[i]);
        OUTPUT_SERIAL.print(";");
    }
    OUTPUT_SERIAL.print("\r\n");

    Serial.printf("[SPEC] Transmitted %d bins\n", HISTOGRAM_BINS);
}

FaultFlags SPEC_CheckQuality(Histogram &hist)
{
    FaultFlags flags = hist.faults;

    if (hist.total_counts < MIN_TOTAL_COUNTS)
        flags |= FAULT_LOW_COUNTS;

    uint32_t zeroBins = 0;
    for (int i = 0; i < HISTOGRAM_BINS; ++i)
        if (hist.bins[i] == 0) ++zeroBins;

    if ((float)zeroBins / (float)HISTOGRAM_BINS > MAX_ZERO_BIN_FRACTION)
        flags |= FAULT_SPARSE_BINS;

    hist.faults = flags;

    if (flags != FAULT_NONE) {
        if (hist.strike_count < 255) ++hist.strike_count;
        Serial.printf("[SPEC] Fault flags: 0x%02X  strikes: %d\n",
                      flags, hist.strike_count);
    } else {
        hist.strike_count = 0;
    }

    return flags;
}

void SPEC_HandleFaults(Histogram &hist1, Histogram &hist2)
{
    bool fault1 = (hist1.strike_count >= FAULT_STRIKE_LIMIT);
    bool fault2 = (hist2.strike_count >= FAULT_STRIKE_LIMIT);

    if (!fault1 && !fault2) return;

    Serial.println("[SPEC] FAULT LIMIT REACHED - rebooting both spectrometers.");

    SPEC_SyncReboot();
    SPEC_InitUntilConnected();

    hist1.strike_count = 0;  hist1.faults = FAULT_NONE;
    hist2.strike_count = 0;  hist2.faults = FAULT_NONE;

    Serial.println("[SPEC] Recovery complete.");
}

static char orinBuf[INFERENCE_BUF_LEN];
static size_t orinIdx = 0;

void ORIN_Poll(void)
{
    while (OUTPUT_SERIAL.available())
    {
        int c = OUTPUT_SERIAL.read();
        if (c < 0 || c == '\r') continue;

        if (c == '\n') {
            if (orinIdx > 0) {
                orinBuf[orinIdx] = '\0';
                memcpy(inference, orinBuf, orinIdx + 1);
                Serial.printf("[ORIN] %s\n", inference);
            }
            orinIdx = 0;
        } else if (orinIdx < INFERENCE_BUF_LEN - 1) {
            orinBuf[orinIdx++] = (char)c;
        }
    }
}

Histogram         hist1, hist2;
CombinedHistogram combined;

void SPEC_AcquisitionLoop(void)
{
    SPEC_InitUntilConnected();
    SPEC_SyncReboot();

    while (true)
    {
        SPEC_ReadHistogram1(hist1);
        SPEC_ReadHistogram2(hist2);

        if (hist1.valid) SPEC_LogHistogramSD(hist1, SD_LOG_FILE_1);
        if (hist2.valid) SPEC_LogHistogramSD(hist2, SD_LOG_FILE_2);

        SPEC_CombineHistograms(hist1, hist2, combined);
        SPEC_ScrubNaN(combined);
        SPEC_LogCombinedSD(combined);
        SPEC_TransmitCombined(combined);

        SPEC_CheckQuality(hist1);
        SPEC_CheckQuality(hist2);
        SPEC_HandleFaults(hist1, hist2);

        yield();
    }
}
