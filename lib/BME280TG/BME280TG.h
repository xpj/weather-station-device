
#ifndef BME280TG_H
#define BME280TG_H

#include <BME280I2C.h>

typedef enum {
    TEMP_C = (1),
    TEMP_F = (2),

} temp_unit;

typedef enum {
    // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi
            PA = (0),
    HPA = (1),
    HG = (2),
    ATM = (3),
    BAR = (4),
    TORR = (5),
    NM2 = (6),
    PSI = (7)
} pressure_unit;

typedef struct {
    float temperature;
    temp_unit temperatureUnit;

    float pressure;
    pressure_unit pressureUnit;

    float relative_humidity;
} units_t;

class BME280TG {
public:
    BME280TG();

    void get(units_t *event);
    float getTemperature(units_t units);
    float getPressure(units_t units);
    float getHumidity(units_t units);
private:
    BME280I2C bme280;
};

#endif //BME280TG_H
