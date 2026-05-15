#include "SensorManager.h"
#include "config.h"
#include "pinout_def.h"

HardwareSerial SensorSerial(2);

enum SensorState {
    STATE_REQUEST,
    STATE_WAIT,
    STATE_PROCESS
};

static String sensorInputBuffer = "";
static SensorState currentStep = STATE_REQUEST;
static unsigned long lastStateTime = 0;
static int measurementCount = 0;

static const int MEASUREMENTS_TO_FINISH = 3;
static const unsigned long TIME_READ_DELAY = 500;

void initSensor() {
    SensorSerial.begin(CX_BAUD, SERIAL_8E1, CX_RX_PIN, CX_TX_PIN);
    currentStep = STATE_REQUEST;
    measurementCount = 0;
    sensorInputBuffer = "";
}

static void sendReadQuery() {
    SensorSerial.write(1); // <SOH>
    SensorSerial.print("#0#0#0#");
    SensorSerial.write(3); // <ETX>
}

static float cleanAndConvert(String segment) {
    segment.trim();
    return segment.toFloat();
}

static void resetSensorValues(TelemetryData &data) {
    data.phValue = 0.0;
    data.conductivity = 0.0;
    data.oxygen = 0.0;
    data.temperature = 0.0;
}

static void parseSensorData(String rawData, TelemetryData &data) {
    resetSensorValues(data);

    int start = 0;
    int end = rawData.indexOf('#');

    while (end != -1) {
        String token = rawData.substring(start, end);

        if (token.indexOf("pH") != -1) {
            data.phValue = cleanAndConvert(token);
        }
        else if (token.indexOf("O2") != -1) {
            data.oxygen = cleanAndConvert(token);
        }
        else if (token.indexOf("S/cm") != -1) {
            data.conductivity = cleanAndConvert(token);
            if (data.conductivity > 10) {
                data.conductivity = data.conductivity / 1000.0; // µS/cm -> mS/cm
            }
        }
        else if (token.indexOf("C") != -1 && token.indexOf("CX") == -1) {
            data.temperature = cleanAndConvert(token);
        }

        start = end + 1;
        end = rawData.indexOf('#', start);
    }

    data.isWaterDetected = (data.conductivity > 0.7);
}

static void processIncomingBuffer(TelemetryData &data) {
    while (SensorSerial.available()) {
        char c = SensorSerial.read();
        if (c == 3) { // <ETX>
            sensorInputBuffer.trim();
            if (sensorInputBuffer.length() > 0) {
                sensorInputBuffer.replace("\x01", "");
                sensorInputBuffer.replace("\x02", "");
                parseSensorData(sensorInputBuffer, data);
            }
            sensorInputBuffer = "";
        }
        else if (c != 1 && c != 2) {
            sensorInputBuffer += c;
        }
    }
}

void runSensorLogic(TelemetryData &data) {
    unsigned long now = millis();

    switch (currentStep) {
        case STATE_REQUEST:
            while (SensorSerial.available()) SensorSerial.read();
            sensorInputBuffer = "";

            sendReadQuery();
            lastStateTime = now;
            currentStep = STATE_WAIT;
            break;

        case STATE_WAIT:
            if (now - lastStateTime >= TIME_READ_DELAY) {
                currentStep = STATE_PROCESS;
            }
            break;

        case STATE_PROCESS:
            processIncomingBuffer(data);

            if (!data.isWaterDetected) {
                measurementCount = 0;
                data.isMeasurementFinished = false;
            }
            else {
                measurementCount++;
                if (measurementCount >= MEASUREMENTS_TO_FINISH) {
                    measurementCount = 0;
                    data.isMeasurementFinished = true;
                } else {
                    data.isMeasurementFinished = false;
                }
            }

            currentStep = STATE_REQUEST;
            break;
    }
}
