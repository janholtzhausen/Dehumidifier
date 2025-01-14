#ifndef SENSOR_H
#define SENSOR_H

void setupSensor();
void readSensorData(); // Add this declaration
double getCurrentTemperature();
double getCurrentHumidity();

extern double currentTemp;
extern double currentHumidity;

#endif // SENSOR_H
