#include "AdaSensors.h"

#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

AdaSensors::AdaSensors() {
    connectLightMeter();
    connectBMP180();
}

uint16_t AdaSensors::getLux() {
    return lightMeter.readLightLevel();
}

float AdaSensors::getPressure() {
    float pressure;
    bmp.getPressure(&pressure);

    return pressure;
}

float AdaSensors::getTemperature() {
    float temperature;
    bmp.getTemperature(&temperature);

    return temperature;
}

float AdaSensors::getAltitude() {
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    return bmp.pressureToAltitude(seaLevelPressure, getPressure());
}

void AdaSensors::connectLightMeter() {
    Serial.println("Connecting BH1750 Light Sensor");
    lightMeter.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);
    Serial.println("Connected to BH1750 Light Sensor");
    Serial.println();
}

void AdaSensors::connectBMP180() {
    Serial.println("Connecting BMP180 Pressure and Temperature Sensor");
    if (!bmp.begin()) {
        Serial.print("Ooops, no BMP180 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    Serial.println("Connected to BMP180 Pressure and Temperature Sensor");
    Serial.println();
}