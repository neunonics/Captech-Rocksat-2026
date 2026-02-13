#include <Arduino.h>

//I2C
#define I2C_SCL PB6
#define I2C_SDA PB7

//UART
#define UART4_TX                    PA0
#define UART4_RX                    PA1
#define SPEC_1_TX                   PA2
#define SPEC_1_RX                   PA3
#define SPEC_2_TX                   PA9
#define SPEC_2_RX                   PA10
#define USART3_CTS                  PB13
#define USART3_RTS                  PB14
#define USART3_TX                   PD8
#define USART3_RX                   PB11
#define UART5_TX                    PB12
#define UART5_RX                    PD2

//LED
#define LED_POWER                   PD9
#define LED_HRTBT                   PD10
#define LED_SDACTIVE                PD11
#define LED_ERROR                   PD12
#define LED_COMM                    PD13

//ENABLES
#define ENA_SPECTRO                 PD14
#define ENA_IRIDIUM                 PD15

//SPI
#define SPI_MISO                    PC2
#define SPI_MOSI                    PC3
#define SPI_CS                      PC12
#define SPI_SCK                     PB10

//IRIDIUM
#define IRIDIUM_SYNC_XVR_OUT        PE10
#define IRIDIUM_GNSS_PASSTHROUGH    PE11
#define IRIDIUM_CAP_CHARGE_EN       PE12
#define IRIDIUM_BOOT_SIGNAL         PE13
#define IRIDIUM_IS_TRANSMITTING     PE14
#define IRIDIUM_WAKE                PE15

//JTAG
#define SYS_JTDO_SWO                PB3
#define SYS_JTMS_SWDIO              PA13
#define SYS_JTCK_SWCLK              PA14
#define SYS_JTDI                    PA15

//MISC
#define TE2_ISOLATED                PA8

