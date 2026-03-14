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
            

            File readFile = SD.open("gamma.txt");
            if (readFile) {

                while (readFile.available()) {
                    SerialB.write(readFile.read());
                }
                readFile.close();
            }
        }
    }
}