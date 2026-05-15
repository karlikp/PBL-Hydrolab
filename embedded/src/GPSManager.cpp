#include "GPSManager.h"
#include "config.h"
#include "pinout_def.h"
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

SFE_UBLOX_GNSS myGNSS;

void initGPS() {
    Wire.begin();
    if (myGNSS.begin() == false) {
        Serial.println(F("GPS Error: Check wiring."));
        //while (1);
    }
    myGNSS.setI2COutput(COM_TYPE_UBX);
    myGNSS.setNavigationFrequency(GPS_HZ);
}

void updateGPS(TelemetryData &data) {
    myGNSS.checkUblox(); // Keep buffer clear

    data.latitude = myGNSS.getLatitude();
    data.longitude = myGNSS.getLongitude();

    data.year = myGNSS.getYear();
    data.month = myGNSS.getMonth();
    data.day = myGNSS.getDay();
    data.hour = myGNSS.getHour();
    data.minute = myGNSS.getMinute();
    data.second = myGNSS.getSecond();
}