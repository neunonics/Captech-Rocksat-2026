#include <Arduino.h>
#include "rockblock_9704.h"

#define I_EN  PA4
#define I_BTD PB1


bool isMessagingComplete = false;
uint32_t startTime = millis();
uint32_t timeoutDuration = 120000;

void onMessageSent(const char* topic, int status, void* context) {
    if (status == 1) {
        isMessagingComplete = true;
    }
}

void setup(){
     pinMode(I_EN,  OUTPUT);
     pinMode(I_BTD, INPUT);
    digitalWrite(I_EN, LOW);
}

void loop() {

    // startutp
    digitalWrite(I_EN, HIGH);
    while (digitalRead(I_BTD) == LOW) { delay(100); }

    //send a message async
    isMessagingComplete = false;
    rbSendMessageAsync(0, (const char*)"Hi", 2);

    //wait until message sent
    while (!isMessagingComplete && (millis() - startTime < timeoutDuration)) {
        rbPoll(); 
        delay(10); 
    }
    // shut down
    digitalWrite(I_EN, LOW);
    
    delay(6000);
}
