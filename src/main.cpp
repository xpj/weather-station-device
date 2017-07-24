#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>

#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <BME280I2C.h>

#include "I2CScanner.h"
#include "RTCAdafruit.h"

#include "private.h"

#define PCF8591 (0x90 >> 1)      // Device address = 0
#define PCF8591_DAC_ENABLE 0x40
#define PCF8591_ADC_CH0 0x40
#define PCF8591_ADC_CH1 0x41
#define PCF8591_ADC_CH2 0x42
#define PCF8591_ADC_CH3 0x43

byte adc_value;

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;

const char *topic = MQTT_TOPIC;
const char *server = MQTT_SERVER;

RTCAdafruit *rtclock;

BH1750 lightMeter(0x23);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
BME280I2C bme280;

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


WiFiClient wifiClient;

const byte RAIN_GAUGE_PIN = 14;
volatile byte rainGaugeTicks = 0;
unsigned long rainGaugeTotalTicks = 0;
unsigned long rainGaugeLastTickTime = 0;
unsigned long TICKS_DEBOUNCE = 300;


void callback(char *topic, byte *payload, unsigned int length) {
    // handle message arrived
}

PubSubClient client(server, 1883, callback, wifiClient);

String macToStr(const uint8_t *mac) {
    String result;
    for (int i = 0; i < 6; ++i) {
        result += String(mac[i], 16);
        if (i < 5)
            result += ':';
    }
    return result;
}

void connectMQTT() {
    // Generate client name based on MAC address and last 8 bits of microsecond counter
    String clientName;
    clientName += "esp8266-";
    uint8_t mac[6];
    WiFi.macAddress(mac);
    clientName += macToStr(mac);
    clientName += "-";
    clientName += String(micros() & 0xff, 16);

    Serial.print("Connecting to ");
    Serial.print(server);
    Serial.print(" as ");
    Serial.println(clientName);

    if (client.connect((char *) clientName.c_str())) {
        Serial.println("Connected to MQTT broker");
        Serial.print("Topic is: ");
        Serial.println(topic);

        if (client.publish(topic, "hello from ESP8266")) {
            Serial.println("Publish ok");
        } else {
            Serial.println("Publish failed");
        }
    } else {
        Serial.println("MQTT connect failed");
        Serial.println("Will reset and try again...");
        abort();
    }
}

void connectWifi() {
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("");
}

void connectLightMeter() {
    Serial.println("Connecting BH1750 Light Sensor");
    lightMeter.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);
    Serial.println("Connected to BH1750 Light Sensor");
    Serial.println();
}

void connectBMP280() {
    if (!bme280.begin()) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        while (1);
    }
}

void connectPressureSensor() {
    Serial.println("Connecting BMP180 Pressure and Temperature Sensor");
    if (!bmp.begin()) {
        Serial.print("Ooops, no BMP180 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    Serial.println("Connected to BMP180 Pressure and Temperature Sensor");
    Serial.println();
}

void setupLeds() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

uint16_t getLux() {
    return lightMeter.readLightLevel();
}

void getBMP280Event(units_t *event) {

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

float getAdaPressure() {
    float pressure;
    bmp.getPressure(&pressure);

    return pressure;
}

float getAdaTemperature() {
    float temperature;
    bmp.getTemperature(&temperature);

    return temperature;
}

float getAdaAltitude() {
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    return bmp.pressureToAltitude(seaLevelPressure, getAdaPressure());
}


float getPressure(units_t event) {
    if (event.pressure) {
        return event.pressure;
    }
}

float getHumidity(units_t event) {
    if (event.relative_humidity) {
        return event.relative_humidity;
    }
}

float getTemperature(units_t event) {
    if (event.temperature) {
        return event.temperature;
    }
}


void handleRainGaugeTick() {
    unsigned long rainGaugeTickTime = millis();
    if (rainGaugeTickTime - rainGaugeLastTickTime > TICKS_DEBOUNCE) {
        rainGaugeTicks++;
        Serial.print("+");
    } else {
        Serial.print("#");
    }
    rainGaugeLastTickTime = rainGaugeTickTime;
}

void getRainGaugeTicks() {
    if (rainGaugeTicks > 0) {
        unsigned long rainGaugeCurrentTicks = rainGaugeTicks;
        rainGaugeTotalTicks = rainGaugeTotalTicks + rainGaugeCurrentTicks;

        rainGaugeTicks = 0;

        Serial.print("Rain sensor. Total ticks: ");
        Serial.print(rainGaugeTotalTicks);
        Serial.print(" Current ticks: ");
        Serial.print(rainGaugeCurrentTicks);
        Serial.println();
    }
}

void connectRainGauge() {
    pinMode(RAIN_GAUGE_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(RAIN_GAUGE_PIN), handleRainGaugeTick, CHANGE);
}

void sendMQTTTemp(float temp1, float lux1, float pres1, float rain1) {
    String payload = "{";
    payload += "\"t1\":";
    payload += temp1;
    payload += ",";
    payload += "\"l1\":";
    payload += lux1;
    payload += ",";
    payload += "\"p1\":";
    payload += pres1;
    payload += ",";
    payload += "\"r1\":";
    payload += rain1;
    payload += "}";

    if (client.connected()) {
        Serial.print("Sending payload: ");
        Serial.println(payload);

        if (client.publish(topic, (char *) payload.c_str())) {
            Serial.println("Publish ok");
        } else {
            Serial.println("Publish failed");
        }
    }
}

byte getADC(byte config) {
    Wire.beginTransmission(PCF8591);
    Wire.write(config);
    Wire.endTransmission();
    Wire.requestFrom((int) PCF8591, 2);
    while (Wire.available()) {
        adc_value = Wire.read(); //This needs two reads to get the value.
        adc_value = Wire.read();
    }
    return adc_value;
}

void process() {
    rtclock->printRtc();

    getRainGaugeTicks();

    units_t event280;
    getBMP280Event(&event280);

    Serial.print("Light:       ");
    Serial.print(getLux());
    Serial.println(" lx");
    Serial.print("Pressure:    ");
    Serial.print(getAdaPressure());
    Serial.println(" hPa");
    Serial.print("Temperature: ");
    Serial.print(getAdaTemperature());
    Serial.println(" C");
    Serial.print("Altitude:    ");
    Serial.print(getAdaAltitude());
    Serial.println(" m");

    Serial.print("Press280:    ");
    Serial.print(getPressure(event280));
    Serial.println(" hPa");
    Serial.print("Temp 280:    ");
    Serial.print(getTemperature(event280));
    Serial.println(" C");
    Serial.print("Hum 280:     ");
    Serial.print(getHumidity(event280));
    Serial.println(" %");

    // Serial.print("Rain:        "); Serial.print(getRainSensorValue()); Serial.println(" %");

//    Serial.print("ADC #0:      ");
//    Serial.print(getADC(PCF8591_ADC_CH0));
//    Serial.println(" ");
//    Serial.print("ADC #1:      ");
//    Serial.print(getADC(PCF8591_ADC_CH1));
//    Serial.println(" ");
//    Serial.print("ADC #2:      ");
//    Serial.print(getADC(PCF8591_ADC_CH2));
//    Serial.println(" ");
//    Serial.print("ADC #3:      ");
//    Serial.print(getADC(PCF8591_ADC_CH3));
//    Serial.println(" ");

    // sendMQTTTemp(getTemperature(event), getLux(), getPressure(event), getRainSensorValue());

}

void setup() {
    Serial.begin(9600);

    setupLeds();
    new I2CScanner();
    rtclock = new RTCAdafruit();
    connectWifi();
    connectMQTT();
    connectLightMeter();
    connectPressureSensor();
    connectBMP280();
    connectRainGauge();
}

void loop() {
    process();
    delay(5000);
}
