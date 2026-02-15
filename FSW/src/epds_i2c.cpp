#include <Wire.h>

#define EPDS_ADDRESS 0xDF

uint8_t readRegister(uint8_t devAddress, uint8_t regAddress) {
  uint8_t value = 0;

  // 1. Tell the device which register we want to look at
  Wire.beginTransmission(devAddress);
  Wire.write(regAddress);
  
  // endTransmission(false) sends a "Restart" instead of a "Stop"
  // This keeps the bus active for the subsequent read.
  if (Wire.endTransmission(false) != 0) {
    return 0xFF; // Error: Device didn't ACK
  }

  // 2. Request 1 byte from the device
  Wire.requestFrom(devAddress, (uint8_t)1);

  if (Wire.available()) {
    value = Wire.read();
  }

  return value;
}