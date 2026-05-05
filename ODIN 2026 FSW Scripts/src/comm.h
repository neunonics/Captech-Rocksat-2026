#include "pins.h"
#include "fsw.h"
#include "rockblock_9704.h"

#define COMM_SERIAL Serial2 // Serial port used for communication with RockBLOCK
#define COMM_BAUD_RATE 230400 // Baud rate for communication with RockBLOCK
#define COMM_TOPIC 244U // Cloudloop RAW topic; must be provisioned on the SIM

struct COMM {
    bool ENBL_STATUS; // 0 = DISABLED, 1 = ENABLED
    bool TX_ACTIVE; // 0 = INACTIVE, 1 = ACTIVE
    int messagesSent; // Number of transmissions sent
};

// -- INIT FUNCTIONS -- //
void clearQueue();
void initCOMMStatus(COMM &comm);
bool initCOMM(COMM &comm);
bool commShutDown(COMM &comm);
bool sendMessage(FSW &fsw, COMM &comm, const String &orinLine, const String &hist);