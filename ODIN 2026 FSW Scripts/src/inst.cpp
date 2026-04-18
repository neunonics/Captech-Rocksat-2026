/**
 * inst.cpp
 * Teensy 4.1 – Dual Gamma Spectrometer Interface
 *
 * Protocol assumptions (adjust to match your device's datasheet):
 *   Probe:           send "ID?\r\n"       → expect line starting with "OK"
 *   Reboot:          send "REBOOT\r\n"    → device resets
 *   Request data:    send "GET_HISTOGRAM\r\n"
 *   Response format:
 *       "DATA:\r\n"
 *       "<uint32 count>\r\n"   × HISTOGRAM_BINS
 *       "END\r\n"
 *
 * Spectrometer 1 → Serial6  (UART6, Teensy 4.1 pins 44/45)
 * Spectrometer 2 → Serial3  (UART3, Teensy 4.1 pins 14/15)
 */

#include "inst.h"

/* ─────────────────────────────────────────────────────────────
 * Internal helpers
 * ───────────────────────────────────────────────────────────── */

/**
 * Read one '\n'-terminated line from a HardwareSerial port into buf.
 * Returns SPEC_OK on success, SPEC_ERR_TIMEOUT if no newline within
 * UART_LINE_TIMEOUT_MS, or SPEC_ERR_UART on overflow.
 */
static SpecStatus readLine(HardwareSerial &port, char *buf, size_t bufLen)
{
    size_t   idx       = 0;
    uint32_t deadline  = millis() + UART_LINE_TIMEOUT_MS;

    while (millis() < deadline)
    {
        if (!port.available()) {
            yield();   /* let Teensy background tasks run */
            continue;
        }

        int c = port.read();
        if (c < 0) continue;

        if (c == '\n') {
            buf[idx] = '\0';
            return SPEC_OK;
        }
        if (c == '\r') continue;   /* strip CR */

        if (idx >= bufLen - 1) {
            buf[idx] = '\0';
            return SPEC_ERR_UART;  /* line too long */
        }
        buf[idx++] = (char)c;
    }

    buf[idx] = '\0';
    return SPEC_ERR_TIMEOUT;
}

/**
 * Attempt a single ID handshake on the given port.
 * Returns true if "OK" prefix is received.
 */
static bool tryConnect(HardwareSerial &port)
{
    /* Discard any stale RX bytes */
    while (port.available()) port.read();

    port.print("ID?\r\n");

    char resp[32];
    SpecStatus st = readLine(port, resp, sizeof(resp));
    if (st != SPEC_OK) return false;

    return (strncmp(resp, "OK", 2) == 0);
}

/**
 * Read a full histogram block from the given serial port into dst.
 *
 * Sends "GET_HISTOGRAM\r\n", waits for "DATA:" header, reads
 * HISTOGRAM_BINS lines, then consumes the "END" trailer.
 */
static SpecStatus readHistogramFrom(HardwareSerial &port, Histogram &dst)
{
    char line[32];
    SpecStatus st;

    memset(&dst, 0, sizeof(dst));
    dst.valid  = false;
    dst.faults = FAULT_NONE;

    /* ── Request ── */
    port.print("GET_HISTOGRAM\r\n");

    /* ── Wait for DATA: header (allow a few lines of latency) ── */
    bool headerFound = false;
    for (int attempt = 0; attempt < 10; ++attempt)
    {
        st = readLine(port, line, sizeof(line));
        if (st == SPEC_ERR_TIMEOUT) {
            dst.faults |= FAULT_UART;
            return SPEC_ERR_TIMEOUT;
        }
        if (strncmp(line, "DATA:", 5) == 0) {
            headerFound = true;
            break;
        }
    }

    if (!headerFound) {
        dst.faults |= FAULT_UART;
        return SPEC_ERR_PARSE;
    }

    /* ── Read HISTOGRAM_BINS values ── */
    dst.total_counts = 0;

    for (int i = 0; i < HISTOGRAM_BINS; ++i)
    {
        st = readLine(port, line, sizeof(line));
        if (st != SPEC_OK) {
            dst.faults |= FAULT_UART;
            return st;
        }

        /* Catch literal NaN / Inf tokens emitted by some firmware */
        if (strncasecmp(line, "nan", 3) == 0 ||
            strncasecmp(line, "inf", 3) == 0)
        {
            dst.bins[i]  = 0;
            dst.faults  |= FAULT_NAN_RAW;
            continue;
        }

        /* Parse unsigned integer */
        char    *endptr = nullptr;
        long     val    = strtol(line, &endptr, 10);

        if (endptr == line) {
            /* Completely unparseable */
            dst.bins[i]  = 0;
            dst.faults  |= FAULT_NAN_RAW;
            continue;
        }

        if (val < 0) val = 0;
        if (val > 0x00FFFFFFL) dst.faults |= FAULT_OVERFLOW;

        dst.bins[i]       = (uint32_t)val;
        dst.total_counts += (uint32_t)val;
    }

    /* ── Consume END marker (non-fatal if missing / timed-out) ── */
    readLine(port, line, sizeof(line));

    dst.valid = true;
    return SPEC_OK;
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

    Serial.println("[SPEC] Waiting for spectrometers...");

    bool s1 = false, s2 = false;

    while (!s1 || !s2)
    {
        if (!s1) {
            s1 = tryConnect(SPEC1_SERIAL);
            if (s1) Serial.println("[SPEC] Spectrometer 1 connected (Serial6).");
        }
        if (!s2) {
            s2 = tryConnect(SPEC2_SERIAL);
            if (s2) Serial.println("[SPEC] Spectrometer 2 connected (Serial3).");
        }

        if (!s1 || !s2) delay(CONNECT_RETRY_DELAY_MS);
    }

    Serial.println("[SPEC] Both spectrometers ready.");
}

void SPEC_SyncReboot(void)
{
    /*
     * Write both reboot commands back-to-back. At 115200 baud each
     * 9-byte frame takes ~0.78 ms, giving <1 ms inter-device skew —
     * negligible relative to typical spectrometer boot time (≥100 ms).
     *
     * For tighter synchronisation, drive a shared GPIO reset line instead.
     */
    SPEC1_SERIAL.print("REBOOT\r\n");
    SPEC2_SERIAL.print("REBOOT\r\n");

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

        /* Guard against any float arithmetic weirdness */
        if (!isfinite(combined)) combined = 0.0f;

        dst.bins[i]       = combined;
        dst.total_counts += (uint32_t)combined;
    }
}

void SPEC_ScrubNaN(CombinedHistogram &hist)
{
    hist.total_counts = 0;

    for (int i = 0; i < HISTOGRAM_BINS; ++i)
    {
        if (isnan(hist.bins[i]) || isinf(hist.bins[i]))
            hist.bins[i] = 0.0f;

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
    OUTPUT_SERIAL.print("DATA:\r\n");

    for (int i = 0; i < HISTOGRAM_BINS; ++i) {
        OUTPUT_SERIAL.println((uint32_t)hist.bins[i]);
    }

    OUTPUT_SERIAL.print("END\r\n");
}

FaultFlags SPEC_CheckQuality(Histogram &hist)
{
    FaultFlags flags = hist.faults;  /* carry any parse-time flags */

    /* ── 1. Low total counts ── */
    if (hist.total_counts < MIN_TOTAL_COUNTS)
        flags |= FAULT_LOW_COUNTS;

    /* ── 2. Sparse bins ── */
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
        hist.strike_count = 0;  /* clear on clean read */
    }

    return flags;
}

void SPEC_HandleFaults(Histogram &hist1, Histogram &hist2)
{
    bool fault1 = (hist1.strike_count >= FAULT_STRIKE_LIMIT);
    bool fault2 = (hist2.strike_count >= FAULT_STRIKE_LIMIT);

    if (!fault1 && !fault2) return;

    Serial.println("[SPEC] FAULT LIMIT REACHED – rebooting both spectrometers.");

    /* Reboot both to keep them in sync regardless of which one faulted */
    SPEC_SyncReboot();

    /* Re-init blocks until both respond */
    SPEC_InitUntilConnected();

    /* Clear fault state */
    hist1.strike_count = 0;  hist1.faults = FAULT_NONE;
    hist2.strike_count = 0;  hist2.faults = FAULT_NONE;

    Serial.println("[SPEC] Recovery complete.");
}

Histogram        hist1, hist2;
 CombinedHistogram combined;

void SPEC_AcquisitionLoop(void)
{

    /* ── Step 1: Connect (blocks until both respond) ── */
    SPEC_InitUntilConnected();

    /* ── Step 2: Sync reboot ── */
    SPEC_SyncReboot();

    /* ── Acquisition super-loop ── */
    while (true)
    {
        /* ── Step 3: Read histograms ── */
        SPEC_ReadHistogram1(hist1);
        SPEC_ReadHistogram2(hist2);

        /* ── Step 3a: Log raw histograms to SD ── */
        if (hist1.valid) SPEC_LogHistogramSD(hist1, SD_LOG_FILE_1);
        if (hist2.valid) SPEC_LogHistogramSD(hist2, SD_LOG_FILE_2);

        /* ── Step 4: Combine ── */
        SPEC_CombineHistograms(hist1, hist2, combined);

        /* ── Step 5: Scrub NaN / Inf ── */
        SPEC_ScrubNaN(combined);

        /* ── Log combined to SD ── */
        SPEC_LogCombinedSD(combined);

        /* ── Transmit combined histogram ── */
        SPEC_TransmitCombined(combined);

        /* ── Step 6: Fault detection ── */
        SPEC_CheckQuality(hist1);
        SPEC_CheckQuality(hist2);
        SPEC_HandleFaults(hist1, hist2);

        /*
         * Optional inter-acquisition delay.
         * Remove or adjust to match your spectrometer's output cadence.
         * yield() keeps the Teensy USB/serial stack responsive.
         */
        yield();
    }
}