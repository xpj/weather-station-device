
#ifndef I2CScanner_h
#define I2CScanner_h

#define SCL_PIN D1
#define SDA_PIN D2

class I2CScanner {
public:
  I2CScanner();
  void printI2CDevices(int sclPin, int sdaPin);
};

#endif
