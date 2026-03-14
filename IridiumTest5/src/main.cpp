#include <Arduino.h>
#include "rockblock_9704.h"

#define I_EN  PA4
#define I_BTD PB1

void setup(){
     pinMode(I_EN,  OUTPUT);
     pinMode(I_BTD, INPUT);
    digitalWrite(I_EN, LOW);
}

void loop()
{
    digitalWrite(I_EN, LOW);
    delay(1000);
    digitalWrite(I_EN, HIGH);
    delay(1000);
}
