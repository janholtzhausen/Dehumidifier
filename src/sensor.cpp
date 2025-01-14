#include "sensor.h"
#include <DHT.h>

#define DHTPIN 15
#define DHTTYPE DHT22
#define MIN_READ_INTERVAL 2100  // Minimum interval between reads

DHT dht(DHTPIN, DHTTYPE);

double currentTemp = 0.0;
double currentHumidity = 0.0;
unsigned long lastReadTime = 0;

void setupSensor() {
    dht.begin();
    Serial.println("DHT22 sensor initialized.");
}

void readSensorData() {
    unsigned long currentTime = millis();

    if (currentTime - lastReadTime >= MIN_READ_INTERVAL) {
        double temp = dht.readTemperature();
        double humidity = dht.readHumidity();

        if (!isnan(temp)) {
            currentTemp = temp;
        } else {
            Serial.println("Failed to read temperature from DHT22 sensor!");
        }

        if (!isnan(humidity)) {
            currentHumidity = humidity;
        } else {
            Serial.println("Failed to read humidity from DHT22 sensor!");
        }

        lastReadTime = currentTime;
    }
}

double getCurrentTemperature() {
    return currentTemp;
}

double getCurrentHumidity() {
    return currentHumidity;
}
