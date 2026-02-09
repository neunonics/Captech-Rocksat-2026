# PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:blackpill_f401ce]
platform = ststm32
board = blackpill_f401ce
framework = arduino
#include <Arduiono.h>
#include <Wire.h>

#//Create data collection

#write.to or read.txt file
#Have SD card save the test .txt file that has the current info of Circuit Health Status
# Create function that reads the Circuit health Status or called Circuit Health Check
#Create another function that reads the status of the antenna's health or data collection 




##void_setup(void){}

#include<stdio.h> 

int main(){
    FILE *pointer = fopen("Testing_file_for_ODIN_to_read_stats.txt");

    char bufferarray[300]; # reading one line at a time from the .txt file
    fgets(bufferarray, 300, pointer);
    printf("%s", bufferarray);

    
    fclose(pointer);
    return 0;

}
#Then closing the while loop:




#void loop(void) {

#}