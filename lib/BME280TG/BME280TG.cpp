#include "BME280TG.h"

#include <BME280I2C.h>

BME280TG::BME280TG() {
    if (!bme280.begin()) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        while (1);
    }
}

void BME280TG::get(units_t *event) {

    uint8_t pressureUnit(HPA);

    float pressure(NAN), temperature(NAN), relative_humidity(NAN);
    bme280.read(pressure, temperature, relative_humidity, pressureUnit, true);

    memset(event, 0, sizeof(units_t));
    event->pressure = pressure;
    event->pressureUnit = HPA;
    event->temperature = temperature;
    event->temperatureUnit = TEMP_C;
    event->relative_humidity = relative_humidity;
}

float BME280TG::getPressure(units_t event) {
    if (event.pressure) {
        return event.pressure;
    }
}

float BME280TG::getHumidity(units_t event) {
    if (event.relative_humidity) {
        return event.relative_humidity;
    }
}

float BME280TG::getTemperature(units_t event) {
    if (event.temperature) {
        return event.temperature;
    }
}
