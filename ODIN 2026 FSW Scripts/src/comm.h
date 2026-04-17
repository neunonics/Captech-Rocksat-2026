// pin 0 is IRID_TX, pin 1 is IRID_RX for Serial1 to RockBLOCK
// I_EN will be on pin 2, I_BTD will be on pin 3
#define IRID_TX 0
#define IRID_RX 1

#define I_EN 2
#define I_BTD 3
#define XMIT_STAT 4

struct COMM {
    bool ENBL_STATUS; // 0 = DISABLED, 1 = ENABLED
    bool TX_ACTIVE; // 0 = INACTIVE, 1 = ACTIVE
    int messagesSent; // Number of transmissions sent
};