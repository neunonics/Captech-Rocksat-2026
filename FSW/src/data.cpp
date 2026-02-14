#include <ArduinoJson.h>
#include <STM32RTC.h>


#define NUM_BINS 1024 // Adjust this based on your detector's resolution

struct FSW_DATETIME{
    u_int8_t day;
    u_int8_t hour;
    u_int8_t second;
    u_int8_t month;
    u_int8_t miniute;
    u_int8_t year;
};

struct FSW_GAMMA_DATA {
    FSW_DATETIME datetime;
    uint32_t spectrumID;
    uint16_t bins[NUM_BINS];
    bool valid; 
};

struct FSW_STATUS{
    bool enable_spectrometer;
    bool enable_iridium;
    bool led_comms;
    bool led_error;
    bool led_sd;
    bool led_hrtbt;
    int error_code;

};

FSW_GAMMA_DATA fetchSpectrum(int spect, HardwareSerial *uart) {
    FSW_GAMMA_DATA data;
    data.spectrumID = spect;
    data.valid = false;

    // Initialize bins to 0
    for(int i = 0; i < NUM_BINS; i++) data.bins[i] = 0;

    if (uart == nullptr) return data;

    // 1. Clear any junk in the serial buffer before sending command
    while(uart->available()) uart->read();

    // 2. Send the specific command from the help menu
    uart->println("read spectrum"); 

    // 3. Wait for data to start (2-second timeout)
    unsigned long startTime = millis();
    while (uart->available() == 0 && (millis() - startTime < 2000)) {
        delay(1);
    }

    // 4. Parse semicolon-separated values
    int binIndex = 0;
    // We loop while there is data and we haven't exceeded our struct size
    while (binIndex < NUM_BINS) {
        // parseInt reads digits and stops at the semicolon
        long val = uart->parseInt();
        
        data.bins[binIndex] = (uint16_t)val;
        binIndex++;

        // Check if we hit the end of the line (end of spectrum)
        // uart->peek() lets us look at the next char without removing it
        char next = uart->peek();
        if (next == '\n' || next == '\r' || next == -1) {
            // Give it a tiny moment to see if more data is coming
            delay(2); 
            if (!uart->available()) break; 
        }
    }

    // 5. Final check
    if (binIndex > 0) {
        data.valid = true;
        //data.timestamp = millis();
    }

    return data;
}