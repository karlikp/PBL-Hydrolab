#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include "TelemetryData.h"

void initGPS();
void updateGPS(TelemetryData &data);

#endif