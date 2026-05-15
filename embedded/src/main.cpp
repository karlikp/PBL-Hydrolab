// H2O drone firmware — skeleton entry point.
//
// This is intentionally thin: it instantiates the system and lets the
// modules do their work. Anything that wants to happen on every loop
// iteration calls its own tick() — main.cpp does not own behaviour.
//
// During the skeleton phase the radio link runs over USB Serial so the
// developer can type CMD frames in the serial monitor and see EVT
// replies in the same window — no radio hardware needed. When the
// production radio module is wired up, swap the Serial reference here
// for Serial1 (or whatever HW UART the radio sits on) and adjust the
// baud rate to the radio's expected 57600.

#include <Arduino.h>

#include "Clock.h"
#include "MissionControl.h"
#include "FrameCodec.h"
#include "RadioLink.h"

namespace {

constexpr const char* FIRMWARE_VERSION = "0.1.0-skeleton";

// USB Serial @ 115200 during skeleton development. See top-of-file
// note for the production wiring change.
constexpr uint32_t LINK_BAUD = 115200;

RealClock        g_clock;
RadioLink        g_radio(Serial);
MissionControl  g_controller(g_clock, g_radio);

void on_frame(void* /*ctx*/, frame::Type type,
              const char* payload, size_t payload_len) {
    // The drone only acts on CMD frames. TLM and EVT inbound are
    // either noise (we're not the receiver of our own broadcasts) or
    // a different sender — drop them.
    if (type == frame::Type::CMD) {
        g_controller.handle_command(payload, payload_len);
    }
}

}  // namespace

void setup() {
    g_radio.begin(LINK_BAUD);
    delay(200);  // brief settle so the BOOT event isn't lost on hot-attach
    g_radio.on_frame(on_frame, nullptr);
    g_controller.boot(FIRMWARE_VERSION);
}

void loop() {
    g_radio.tick();
    g_controller.tick();
    delay(5);
}
