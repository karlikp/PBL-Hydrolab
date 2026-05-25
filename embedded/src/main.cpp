// H2O drone firmware — top-level composition.
//
// Three build personalities share this file:
//
//   ISOLATE_UART0_FOR_SERVO  Servo-only bring-up. UART0 dedicated to
//                            SC-09 bus, every other UART0 producer
//                            stripped out. Debug via JTAG.
//
//   BOARD_ESP32_S3           Production board. Operator comms over
//                            the RFD868 radio (Serial1, IO10/IO11);
//                            UART0 dedicated to the servo bus with
//                            the swapped TX=44/RX=43 mapping. CP210x
//                            dev header is unused at runtime.
//
//   BOARD_ESP32_CLASSIC      Legacy dev board. No radio, no servo
//                            hardware. Protocol over the CP210x
//                            adapter on Serial (UART0). Useful for
//                            running the FSM / protocol code on the
//                            bench without production hardware.
//
// main.cpp deliberately does no driver work — it constructs the
// modules and wires them up. Pinout / baud / per-peripheral setup
// lives in each module's own class.

#include <Arduino.h>

#ifdef BOARD_ESP32_S3
#  include <WiFi.h>
#  include <esp_bt.h>
#endif

#include "ServoBus.h"
#if !defined(ISOLATE_UART0_FOR_SERVO) && !defined(RADIO_TEST_MODE)
#  include "BatteryMonitor.h"
#  include "Clock.h"
#  include "FrameCodec.h"
#  include "MissionControl.h"
#  include "PumpControl.h"
#  include "RadioLink.h"
#  include "RadioTransport.h"
#  include "Telemetry.h"
#endif

namespace {

// HARDWARE QUIRK — ESP32-S3 production board.
// GPIO 4 holds the soft-power latch. If it's not driven HIGH within
// the first few ms after boot, the latch releases and the whole board
// powers off. setup() must do this BEFORE anything else.
constexpr int POWER_LATCH_PIN = 4;

// Battery voltage sense (divider on the schematic: R18 upper, R19 lower).
constexpr int BATTERY_ADC_PIN = 5;

constexpr const char* FIRMWARE_VERSION = "0.1.0-skeleton";

#ifdef BOARD_ESP32_S3
// Production-board first-cycle init.
//
//   1) Drive the soft-power latch HIGH within the first few ms of
//      boot — if we miss the window the latch releases and the board
//      powers itself off mid-setup().
//   2) Explicitly disable WiFi and Bluetooth. Arduino-ESP32 doesn't
//      auto-start either, but the radio peripherals can still be in
//      a low-power-but-non-zero state until told to shut down. Doing
//      this also releases their internal task/RAM resources.
//
// Call this as the FIRST thing in every S3 build's setup() — before
// Serial.begin, before any other peripheral.
void early_board_init() {
    pinMode(POWER_LATCH_PIN, OUTPUT);
    digitalWrite(POWER_LATCH_PIN, HIGH);
    WiFi.mode(WIFI_OFF);
    btStop();
}
#endif

#if defined(ISOLATE_UART0_FOR_SERVO) || defined(RADIO_TEST_MODE)
// Bring-up builds — strip the operator stack out of the globals.
#else

RealClock g_clock;

#ifdef BOARD_ESP32_S3
// Production: radio is the operator link; UART0 is dedicated to servos.
RadioTransport g_radio_transport(Serial1);
RadioLink      g_radio(g_radio_transport.stream());
ServoBus       g_servo_bus(Serial);
#else
// Classic dev board: protocol over CP210x on Serial. No real radio or
// servo HW present, but ServoBus still constructs cleanly so the same
// MissionControl code path compiles.
constexpr uint32_t DEV_LINK_BAUD = 115200;
RadioLink      g_radio(Serial);
ServoBus       g_servo_bus(Serial);  // unused in classic, no real bus
#endif

MissionControl g_controller(g_clock, g_radio);
Telemetry      g_telemetry(g_clock, g_radio);
#ifdef BOARD_ESP32_S3
BatteryMonitor g_battery(g_clock, g_radio, BATTERY_ADC_PIN, POWER_LATCH_PIN);
PumpControl    g_pumps;
#endif

void on_frame(void* /*ctx*/, frame::Type type,
              const char* payload, size_t payload_len) {
    if (type == frame::Type::CMD) {
        g_controller.handle_command(payload, payload_len);
    }
}
#endif

}  // namespace

#ifdef RADIO_TEST_MODE

// =====================================================================
// RADIO BRING-UP DIAGNOSTIC FIRMWARE
// =====================================================================
// One flash, two answers:
//
//   (1) Is the firmware running at all? — `pio device monitor` on
//       /dev/ttyUSB0 at 115200 shows a heartbeat from Serial.
//
//   (2) Which TX/RX orientation reaches the V5? — every 5 seconds
//       the firmware tears down Serial1 and reopens it with the
//       other pin orientation. While each orientation is active it
//       sends "PING <n>\n" at the radio's baud. Whichever orientation
//       shows up on the host V5 (listening at 57600) is the correct
//       one.
//
// Anything received on Serial1 is echoed back to the debug log on
// Serial, so we also know if the V5 is replying or if the host side
// is leaking back.
//
// Flash:   pio run -e esp32s3wroom1-radiotest -t upload
// Debug:   pio device monitor -p /dev/ttyUSB0 -b 115200
// Radio:   listen on V5 tty at 57600 for "PING N"
// =====================================================================

namespace {
constexpr uint32_t RADIO_BAUD          = 57600;
constexpr uint32_t SWAP_INTERVAL_MS    = 5000;
constexpr uint32_t PING_INTERVAL_MS    = 500;
constexpr int      RFD_IO_A            = 10;  // board net RFD_RX
constexpr int      RFD_IO_B            = 11;  // board net RFD_TX
}  // namespace

void setup() {
    early_board_init();

    // CP210x debug log uses standard UART0 pin map at 115200. Boot
    // log will appear on /dev/ttyUSB0 immediately.
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("=== radio bring-up diagnostic ===");
    Serial.printf("RFD_IO_A=%d RFD_IO_B=%d baud=%u\n",
                  RFD_IO_A, RFD_IO_B, RADIO_BAUD);
}

void loop() {
    static bool      use_swap     = false;
    static uint32_t  last_swap_ms = 0;
    static uint32_t  last_ping_ms = 0;
    static uint32_t  counter      = 0;
    static bool      first        = true;

    const uint32_t now = millis();

    if (first || (now - last_swap_ms) >= SWAP_INTERVAL_MS) {
        first        = false;
        last_swap_ms = now;
        use_swap     = !use_swap;

        Serial1.end();
        // orientation A: chip TX = RFD_IO_A (IO10), chip RX = RFD_IO_B (IO11)
        // orientation B: chip TX = RFD_IO_B (IO11), chip RX = RFD_IO_A (IO10)
        const int rx = use_swap ? RFD_IO_A : RFD_IO_B;
        const int tx = use_swap ? RFD_IO_B : RFD_IO_A;
        Serial1.begin(RADIO_BAUD, SERIAL_8N1, rx, tx);
        Serial.printf("\n[swap] now using ESP TX=IO%d RX=IO%d (orientation %s)\n",
                      tx, rx, use_swap ? "B" : "A");
    }

    if ((now - last_ping_ms) >= PING_INTERVAL_MS) {
        last_ping_ms = now;
        ++counter;
        Serial1.printf("PING %lu\n", static_cast<unsigned long>(counter));
        Serial.printf("[tx] PING %lu  (orientation %s)\n",
                      static_cast<unsigned long>(counter),
                      use_swap ? "B" : "A");
    }

    // Echo anything coming back from the V5 to the debug log.
    while (Serial1.available()) {
        const int b = Serial1.read();
        if (b >= 0) {
            Serial.printf("[rx] %02X '%c'\n",
                          static_cast<unsigned>(b),
                          (b >= 32 && b < 127) ? static_cast<char>(b) : '.');
        }
    }
}

#elif defined(ISOLATE_UART0_FOR_SERVO)

// =====================================================================
// SERVO TEST FIRMWARE
// =====================================================================
// Standalone bring-up build for the SC-09 servo bus. Does nothing
// except open UART0 with the SWAPPED pin mapping and rock servo ID=1
// between two positions through ServoBus.
//
// Flash:  pio run -e esp32s3wroom1-servotest -t upload
// =====================================================================

HardwareSerial& servo_serial = Serial;
ServoBus servos(servo_serial);

void setup() {
    early_board_init();

    servo_serial.begin(ServoBus::DEFAULT_BAUD, SERIAL_8N1,
                       ServoBus::RX_PIN, ServoBus::TX_PIN);

    delay(2000);  // SC-09 servos finish their own boot
}

void loop() {
    // Demo: rock the servo at ID=1 between two positions.
    // Fresh-from-factory servos all share ID=1, so this is the right
    // address until we program unique per-tank IDs.
    servos.move(1, 200);
    delay(1000);
    servos.move(1, 800);
    delay(1000);
}

#else

void setup() {
#ifdef BOARD_ESP32_S3
    // Power latch + WiFi/BT off — must run before anything else.
    early_board_init();

    // UART0 → servo bus (swapped TX=44/RX=43). Operator comms is on
    // the radio (Serial1), set up just below.
    Serial.begin(ServoBus::DEFAULT_BAUD, SERIAL_8N1,
                 ServoBus::RX_PIN, ServoBus::TX_PIN);

    g_radio_transport.begin();
#else
    // Classic dev board: just open the CP210x link.
    Serial.begin(DEV_LINK_BAUD);
#endif

    delay(200);  // settle so the BOOT event isn't lost on hot-attach
    g_radio.on_frame(on_frame, nullptr);

#ifdef BOARD_ESP32_S3
    g_battery.begin();
    g_pumps.begin();
    g_controller.set_battery_monitor(&g_battery);
    g_controller.set_servo_bus(&g_servo_bus);
    g_controller.set_pump_control(&g_pumps);
#endif

    g_controller.boot(FIRMWARE_VERSION);
}

void loop() {
    g_radio.tick();
    g_controller.tick();
    g_telemetry.tick();
#ifdef BOARD_ESP32_S3
    g_battery.tick();
#endif
    delay(5);
}

#endif  // ISOLATE_UART0_FOR_SERVO
