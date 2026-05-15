#ifndef RADIO_MANAGER_H
#define RADIO_MANAGER_H

#include "TelemetryData.h"

void initRadio();
void broadcastData(const TelemetryData &data);

#endif