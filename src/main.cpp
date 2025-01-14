#include <Arduino.h>
#include <WiFi.h>
#include <web.h>
#include <shared.h>
#include <heater.h>
#include <PID_v1.h>
#include <config.h>
#include "sensor.h"
#include <esp_task_wdt.h>

#define PID_INTERVAL 200
#define WDT_TIMEOUT 30

double gTargetTemp=S_TSET;
double gOvershoot=S_TBAND;
double gOutputPwr=0.0;
double gP = S_P, gI = S_I, gD = S_D;
double gaP = S_aP, gaI = S_aI, gaD = S_aD;

boolean overShootMode = false;
unsigned long time_now=0;
unsigned long time_last=0;

PID ESPPID(&currentTemp, &gOutputPwr, &gTargetTemp, gP, gI, gD, DIRECT);


void setup() {
  Serial.begin(115200);
  esp_task_wdt_init(WDT_TIMEOUT, true); // Initialize ESP32 Task WDT
  esp_task_wdt_add(NULL);               // Subscribe to the Task WDT

  Serial.println("Mounting LittleFS...");
  if(!prepareFS()) {
    Serial.println("Failed to mount LittleFS !");
  } else {
    Serial.println("Mounted.");
  }
  Serial.println("Loading config...");
  if (!loadConfig()) {
    Serial.println("Failed to load config. Using default values and creating config...");
    if (!saveConfig()) {
     Serial.println("Failed to save config");
    } else {
      Serial.println("Config saved");
    }
  } else {
    Serial.println("Config loaded");
  }

    // Hardcoded Wi-Fi credentials
    const char* ssid = "yourssid";       // Replace with your Wi-Fi SSID
    const char* password = "yourssidpassword"; // Replace with your Wi-Fi Password

    // Connect to Wi-Fi
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nConnected to Wi-Fi.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());



  setupWeb();
  setupSensor();
  setupHeater();

  // start PID
  ESPPID.SetTunings(gP, gI, gD);
  ESPPID.SetSampleTime(PID_INTERVAL);
  ESPPID.SetOutputLimits(0, 1000);
  ESPPID.SetMode(AUTOMATIC);

  time_now=millis();
  time_last=time_now;
}

void logFreeHeap() {
    Serial.print("Free Heap: ");
    Serial.println(esp_get_free_heap_size());
}

void loop() {
    
    logFreeHeap();
    delay(1000); // Log heap every second

    time_now = millis();

    // Use global `currentTemp` updated by `getCurrentTemperature()`
    // currentTemp = getCurrentTemperature();
    // currentHumidity = getCurrentHumidity();

    readSensorData();

    // Direct comparison for unsigned time difference
    if ((time_now - time_last >= PID_INTERVAL) || (time_last > time_now)) {
        if (!overShootMode && fabs(gTargetTemp - currentTemp) >= gOvershoot) {
            ESPPID.SetTunings(gaP, gaI, gaD);
            overShootMode = true;
        } else if (overShootMode && fabs(gTargetTemp - currentTemp) < gOvershoot) {
            ESPPID.SetTunings(gP, gI, gD);
            overShootMode = false;
        }
        if (ESPPID.Compute() == true) {
            setHeatPowerPercentage(gOutputPwr);
        }
        time_last = time_now;
    }

    updateHeater();

    // Kick the dog
    esp_task_wdt_reset();
}

