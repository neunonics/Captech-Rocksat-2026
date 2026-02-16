
#include "data.h"

void flagsToJson(JsonDocument& json, FSW_DATETIME datetime){
JsonObject dt = json.createNestedObject("datetime");
dt["hour"] = datetime.hour;
dt["second"] = datetime.second;
dt["minute"] = datetime.minute;
dt["day"] = datetime.day;
dt["month"] = datetime.month;
dt["year"] = datetime.year;
return;
}

int saveData(JsonDocument json, String filename){
    File file = SD.open(filename, FILE_WRITE);
    if (file) {
        serializeJson(json, file);
        file.println();
        file.close();
        return 0;
    }

    return 1;
}

FSW_GAMMA_DATA fetchSpectrum(int spect, HardwareSerial *uart, FSW_DATETIME dt) {
    FSW_GAMMA_DATA data;
    data.spectrumID = spect;
    data.valid = false;

    // Initialize bins to 0
    for(int i = 0; i < NUM_BINS; i++) data.bins[i] = 0;

    if (uart == nullptr) return data;

    // 1. Clear any junk in the serial buffer before sending command
    while(uart->available()) uart->read();

    uart->setTimeout(2000);

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
        data.datetime = dt;
    }

    uart->setTimeout(1000);

    return data;
}