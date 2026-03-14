#include <SPI.h>
#include <SD.h>

HardwareSerial SerialA(PA10, PA9); // High Speed (2,000,000)
HardwareSerial SerialB(PA3, PA2);  // Low Speed (9,600)

const int chipSelect = PA4;

void setup() {
    SerialA.begin(2000000); 
    SerialB.begin(9600);
    SD.begin(chipSelect);
}

void loop() {
    // 1. CATCH THE FIREHOSE
    if (SerialA.available()) {
        File dataFile = SD.open("gamma.txt", FILE_WRITE);
        
        if (dataFile) {
            // Read as fast as possible until the newline
            while (true) {
                if (SerialA.available()) {
                    char c = SerialA.read();
                    dataFile.write(c); // Write to SD buffer
                    
                    if (c == '\n') break; 
                }
            }
            dataFile.close(); // Save to SD
            
            // 2. BLEED TO THE STRAW
            // Now that the data is safe on the SD, we read it back 
            // and send it slowly to Serial B
            File readFile = SD.open("gamma.txt");
            if (readFile) {
                // Seek to the start of the last entry or just send the whole file
                // For this example, we'll just send the line we just wrote
                // (This part requires tracking file offsets for efficiency)
                while (readFile.available()) {
                    SerialB.write(readFile.read());
                    // Serial B is so slow, we don't even need a delay; 
                    // the HardwareSerial buffer will handle the pacing.
                }
                readFile.close();
            }
        }
    }
}