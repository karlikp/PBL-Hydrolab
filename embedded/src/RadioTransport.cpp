#include "RadioTransport.h"

#include <Arduino.h>
#include <HardwareSerial.h>

void RadioTransport::begin() {
    serial_.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
}
