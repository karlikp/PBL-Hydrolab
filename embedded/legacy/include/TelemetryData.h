#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

#include <Arduino.h>

struct TelemetryData {
    // GPS
    long latitude;
    long longitude;

    // Time
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    // Sensor (CX-401)
    float conductivity;
    float temperature;
    float phValue;
    float oxygen;

    // Logic
    bool isWaterDetected;
    bool isMeasurementFinished; // True when all 3 sensors have finished their 3-reading cycles
};

#endif