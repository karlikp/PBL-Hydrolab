#include "RadioManager.h"
#include "config.h"
#include "pinout_def.h"
HardwareSerial RadioSerial(1);

void initRadio() {
    // Verified: 57600 baud, 8N1 is the correct setting
    RadioSerial.begin(RADIO_BAUD, SERIAL_8N1, RADIO_RX_PIN, RADIO_TX_PIN);
}

// --- Adler-32 Algorithm ---
uint32_t adler32(const char *data, size_t len) {
     uint32_t a = 1, b = 0;
     for (size_t i = 0; i < len; i++) {
         a = (a + data[i]) % 65521;
         b = (b + a) % 65521;
     }
     return (b << 16) | a;
}

void broadcastData(const TelemetryData &data) {
    char buff[200];
    
    // Format: Time,Lat,Lon,Cond,Temp,pH,O2,WaterFlag
    // Note: Lat/Lon are raw integers here (e.g. 50123456)
    snprintf(buff, sizeof(buff), "%04d-%02d-%02d %02d:%02d:%02d,%d,%d,%.2f,%.2f,%.2f,%.2f,%d", 
             data.year, data.month, data.day,
             data.hour, data.minute, data.second,
             data.latitude, 
             data.longitude, 
             data.conductivity, 
             data.temperature, 
             data.phValue, 
             data.oxygen,
             data.isWaterDetected);

    // Compute Adler-32 Checksum
    uint32_t checksum = adler32(buff, strlen(buff));

    // Output: Data + "*" + 8-Digit Hex Checksum
    // Example: ... ,1*04A1B2C3
    
    // To USB (Debug)
    Serial.print(buff);
    Serial.print("*");
    Serial.printf("%08X\n", checksum); // %08X ensures 8 chars fixed width

        // To Radio (Telemetry)
    RadioSerial.print(buff);
    RadioSerial.print("*");
    RadioSerial.printf("%08X\n", checksum);
}