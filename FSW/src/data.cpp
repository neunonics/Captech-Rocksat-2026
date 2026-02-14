#include <ArduinoJson.h>
#include <STM32RTC.h>


#define NUM_BINS 1024 // Adjust this based on your detector's resolution

struct FSW_DATETIME{
    u_int8_t day;
    u_int8_t hour;
    u_int8_t second;
    u_int8_t month;
    u_int8_t minute;
    u_int8_t year;
    uint32_t subseconds;
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
    bool is_armed;
    bool is_te2;

    bool spect_1_connected;
    bool spect_2_connected;
    bool orin_connected;
    bool comms_connected;
    bool bno1_connected;
    bool bno2_connected;

};

struct FSW_IMU_EULER{
    float roll;
    float pitch;
    float yaw;
};

struct FSW_IMU_XYZ{
    float x;
    float y;
    float z;
};

struct FSW_IMU_QUAT{
    float q1;
    float q2;
    float q3;
    float q4;
};

struct FSW_IMU{
    FSW_IMU_EULER euler;
    FSW_IMU_XYZ bfield;
    FSW_IMU_XYZ gravity;
    FSW_IMU_QUAT quaternion;
};

struct FSW_ADS{
    FSW_IMU imu1;
    FSW_IMU imu2;
};

struct FSW_PREDICTION{
    int priority;
    int spectrumID;
    String prediction;
    float probability;
};

struct FSW_POWER{
    float voltage;
    float current;
    float power;
};

struct FSW_EDS{
    FSW_POWER twelveV;
    FSW_POWER fiveV;
    FSW_POWER threeV;

};

FSW_GAMMA_DATA fetchSpectrum(int spect, HardwareSerial *uart, STM32RTC *rtc) {
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
        data.datetime.second = rtc->getSeconds();
        data.datetime.minute = rtc->getMinutes();
        data.datetime.hour   = rtc->getHours();
        data.datetime.day     = rtc->getDay();
        data.datetime.month   = rtc->getMonth();
        data.datetime.year    = rtc->getYear();
    }

    uart->setTimeout(1000);

    return data;
}