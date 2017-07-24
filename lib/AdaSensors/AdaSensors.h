
#ifndef ADASENSORS_H
#define ADASENSORS_H

#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

class AdaSensors {

public:
    AdaSensors();

    uint16_t getLux();
    float getPressure();
    float getTemperature();
    float getAltitude();

private:
    BH1750 lightMeter = BH1750(0x23);
    Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

    void connectLightMeter();
    void connectBMP180();
};

#endif //ADASENSORS_H
