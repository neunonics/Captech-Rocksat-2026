#include <Arduino.h>

// -- FSW -- //
#define TE2_SIGNAL          45  // RockSat TE-2 Signal to Start Science Operations (23_19_CRX1_MCLK1)

#define LED_PWR             13  // LED Pin to Indiciate Powered (11_MOSI_CTX1)
#define LED_HRTBT           14 // LED Pin to Indiciate Heartbeat (12_MISO_MQSL)
#define LED_SDACTIVE        38 // LED Pin to Indicate SD Card Active (16_A2_RX4_SCL1)
#define LED_COMM            18 // LED Pin to Indicate COMM Active (26_A12_MOSI1)
#define LED_ERROR           39 // LED Pin to Indicate Error has Occured (17_A3_TX4_SDA1)

#define I2C_SCL             41  // Serial Clock Pin for I2C (19_A5_SCL)
#define I2C_SDA             40  // Serial Data Pin for I2C (18_A4_SDA)

#define DEBUG_UART_TX       2 // Debug UART Transmit (1_TX1_CTX2_MIS01)
#define DUBUG_UART_RX       1 // Debug UART Receive (0_RX1_CRX2_CS1)

// -- SD CARD -- //
//#define SPI_MISO            PC_2  // 
//#define SPI_MOSI            PC_3  //
//#define SPI_CS              PC_12 //
//#define SPI_SCK             PB10 //

// -- EPDS -- //

// -- INST -- //
#define SPEC_EN             19 // Pin to Enable Spectrometers (27_A13_SCK1)
#define SPEC_A_TX           16   // SPEC_A Transmit (24_A10_TX6_SCL2)
#define SPEC_A_RX           17   // SPEC_A Receive (25_A11_RX6_SDA2)
#define SPEC_B_TX           37   // SPEC_B Transmit (15_A1_RX3_SPDIF_IN)
#define SPEC_B_RX           36 // SPEC_B Receive (14_A0_TX3_SPDIF_OUT)

#define JETSON_UART_CTS     8 // JETSON Clear to Send (6_OUT1D)
#define JETSON_UART_RTS     7 // JETSON Ready to Send (5_IN2)
#define JETSON_TX           43 // JETSON Transmit (21_A7_RX5_BCLK1)
#define JETSON_RX           42 // JETSON Recieve (20_A6_TX5_LRCLK1)

// -- COMM -- //
#define COMM_TX             9 // FSW -> COMM (7_RX2_OUT1A)
#define COMM_RX             10 // COMM -> (8_TX2_IN1)
#define COMM_EN             44 // COMM Enable Pin (22_A8_CTX1)
#define COMM_BTD            35 // COMM Booted Pin (13_SCK_LED)
#define XMIT_STAT           5 // COMM Transmit (3_LRCLK2)