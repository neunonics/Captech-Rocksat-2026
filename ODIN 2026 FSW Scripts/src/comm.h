// pin 0 is IRID_TX, pin 1 is IRID_RX for Serial1 to RockBLOCK
// I_EN will be on pin 2, I_BTD will be on pin 3

#include "pins.h"

struct COMM {
    bool ENBL_STATUS; // 0 = DISABLED, 1 = ENABLED
    bool TX_ACTIVE; // 0 = INACTIVE, 1 = ACTIVE
    int messagesSent; // Number of transmissions sent
};

// -- INIT FUNCTIONS -- //
void initCOMMStatus(COMM &comm);
bool initCOMM(COMM &comm);
bool commShutDown(COMM &comm);