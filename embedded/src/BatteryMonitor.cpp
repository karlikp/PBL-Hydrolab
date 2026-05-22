#include "BatteryMonitor.h"

#include <Arduino.h>
#include <stdio.h>

#include "Clock.h"
#include "RadioLink.h"

BatteryMonitor::BatteryMonitor(Clock& clock, RadioLink& radio,
                               int adc_pin, int latch_pin)
    : clock_(clock), radio_(radio),
      adc_pin_(adc_pin), latch_pin_(latch_pin) {}

void BatteryMonitor::begin() {
    // Force the pin to a clean analog-input state. pinMode(INPUT) is
    // SUPPOSED to disable internal pull-ups/downs, but on some
    // Arduino-ESP32 builds the ROM/framework leaves a residual config.
    // Be explicit just in case.
    pinMode(adc_pin_, INPUT);
    gpio_pullup_dis(static_cast<gpio_num_t>(adc_pin_));
    gpio_pulldown_dis(static_cast<gpio_num_t>(adc_pin_));
    // 11 dB attenuation gives the widest input range (~0–3.1 V on S3).
    analogSetPinAttenuation(adc_pin_, ADC_11db);
    // latch_pin_ is set OUTPUT + HIGH in main.cpp before we get here;
    // we only drive it LOW on shutdown.
}

void BatteryMonitor::tick() {
    const uint32_t now = clock_.now_ms();
    if (now - last_sample_ms_ < SAMPLE_INTERVAL_MS) return;
    last_sample_ms_ = now;

    last_voltage_ = read_voltage();

    if (state_ == State::OK && last_voltage_ < WARNING_V) {
        state_ = State::WARNING;
        char buf[80];
        snprintf(buf, sizeof(buf), "EVT,SYS,ERROR,low_battery:%.2fV",
                 last_voltage_);
        radio_.send(buf);
    }

    if (state_ != State::CRITICAL && last_voltage_ < CRITICAL_V) {
        state_ = State::CRITICAL;
        shutdown();
    }
}

float BatteryMonitor::read_voltage() {
    last_raw_ = static_cast<uint16_t>(analogRead(adc_pin_));
    last_mv_  = analogReadMilliVolts(adc_pin_);
    // NOTE: previously we had an ADC_CORRECTION multiplier here.
    // That was based on a single calibration data point taken while
    // the ADC was saturated — i.e. it wasn't a calibration, it was
    // an accident. Removed until we have a working raw read.
    const float v_adc = last_mv_ / 1000.0f;
    return v_adc / DIVIDER_RATIO;
}

void BatteryMonitor::shutdown() {
    char buf[80];
    snprintf(buf, sizeof(buf), "EVT,SYS,ERROR,shutdown_low_battery:%.2fV",
             last_voltage_);
    radio_.send(buf);

    // Give the UART a chance to drain the frame before the latch drops
    // and the chip loses power. 50 ms is generous at 115200 baud for a
    // ~40-byte frame (~3.5 ms of actual TX).
    delay(50);

    digitalWrite(latch_pin_, LOW);

    // The board will power off within a few ms. Spin so we don't keep
    // running on residual capacitor charge.
    while (true) { delay(10); }
}
