#include <Arduino.h>

// -- FSW -- //
#define TE2_SIGNAL          PA8  // RockSat TE-2 Signal to Start Science Operations

#define LED_PWR             PD9  // LED Pin to Indiciate Powered
#define LED_HRTBT           PD10 // LED Pin to Indiciate Heartbeat
#define LED_SDACTIVE        PD11 // LED Pin to Indicate SD Card Active
#define LED_COMM            PD12 // LED Pin to Indicate COMM Active
#define LED_ERROR // LED Pin to Indicate Error has Occured

#define I2C_SCL // Serial Clock Pin for I2C
#define I2C_SDA // Serial Data Pin for I2C

#define DEBUG_UART_TX // Debug UART Transmit
#define DUBUG_UART_RX // Debug UARt Receive

// -- SD CARD -- //
#define SPI_MISO // 
#define SPI_MOSI //
#define SPI_CS //
#define SPI_SCK //


// -- EPDS -- //

// -- INST -- //
#define SPEC_EN // Pin to Enable Spectrometers

#define SPEC_A_TX // SPEC_A Transmit
#define SPEC_A_RX // SPEC_A Receive
#define SPEC_B_TX // SPEC_B Transmit
#define SPEC_B_RX // SPEC_B Receive

#define JETSON_UART_CTS // JETSON Clear to Send
#define JETSON_UART_RTS // JETSON Ready to Send
#define JETSON_TX // JETSON Transmit
#define JETSON_RX // JETSON Recieve

// -- COMM -- //
#define COMM_TX // FSW -> COMM (Transmit Pin)
#define COMM_RX // COMM -> (FSW Recieve Pin)
#define COMM_EN // COMM Enable Pin
#define COMM_BTD // COMM Booted Pin
#define XMIT_STAT // COMM Transmit 