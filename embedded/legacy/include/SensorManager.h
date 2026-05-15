#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include "TelemetryData.h"

void initSensor();
void runSensorLogic(TelemetryData &data);

#endif
