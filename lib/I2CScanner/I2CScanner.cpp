#include <I2CScanner.h>

#include <Wire.h>
#include "Arduino.h"

I2CScanner::I2CScanner()
{
  printI2CDevices(SDA_PIN, SCL_PIN);
}

void I2CScanner::printI2CDevices(int sdaPin, int sclPin)
{
  byte error, address;
  int nDevices;

  Wire.begin(sdaPin, sclPin);
  Serial.println();
  Serial.print("I2C Devices: ");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
          // The i2c scanner uses the return value of
          // the Write.endTransmisstion to see if
          // a device did acknowledge to the address.
          Wire.beginTransmission(address);
          error = Wire.endTransmission();

          if (error == 0)
          {
                  Serial.print("[0x");
                  if (address < 16) {
                          Serial.print("0");
                  }
                  Serial.print(address, HEX);
                  Serial.print("|OK]");

                  nDevices++;
          }
          else if (error == 4)
          {
                  Serial.print("[0x");
                  if (address < 16) {
                          Serial.print("0");
                  }
                  Serial.print(address, HEX);
                  Serial.print("|ER]");
          }
  }
  if (nDevices == 0) {
          Serial.println("No I2C devices found\n");
  }
  else {
          Serial.println();
  }
}
