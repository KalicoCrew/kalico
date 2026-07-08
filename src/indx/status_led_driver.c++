#include "status_led_driver.h"
#include <cmath>
#include <numbers>

#define TICK_RATE (1.0f / 100.0f)

status_led_driver::status_led_driver(i2c_bus *bus)
    : bus(bus), good(false), current_pattern(pattern_off, std::nullopt) {

    good = true;
    good &= write_u8(0x0E, 0xCC); // Chip reset
    good &= write_u8(0x00, 0x01); // Chip enable
    good &= write_u8(0x01, 0x00); // Max currrent = 25.5mA
    good &= write_u8(0x02, 0x07); // Enable all LEDs
    good &= write_u8(0x0F, 0x55); // Update command

    // Verify write was good
    uint8_t readback;
    good &= read_u8_into(0x02, readback);
    good &= readback == 0x07;
}

void
status_led_driver::set_currents(uint8_t red, uint8_t green, uint8_t blue) {
    if (!good)
        return;
    good &= write_u8(0x14, red); // Red current
    good &= write_u8(0x15, green); // Green current
    good &= write_u8(0x16, blue); // Blue current
}

void
status_led_driver::force_color(std::optional<color_t> color) {
    this->override_color = color;
    if (this->override_color) {
        good &= write_color(*this->override_color);
    } else {
        auto last_color = this->current_pattern.sample();
        good &= write_color(last_color);
    }
}

bool
status_led_driver::write_u8(uint8_t reg, uint8_t val) {
    uint8_t data[] = {reg, val};
    return bus->transfer(0x2D, data, 2, nullptr, 0);
}

bool
status_led_driver::read_u8_into(uint8_t reg, uint8_t &out) {
    uint8_t data[] = {reg};
    return bus->transfer(0x2D, data, 1, &out, 1);
}

bool
status_led_driver::write_color(color_t color) {
    return write_u8(0x18, color.r) && write_u8(0x19, color.g) &&
           write_u8(0x1A, color.b);
}

void
status_led_driver::set_color(color_t color) {
    this->set_pattern(pattern_t{color, 0, 0, 0.0f, 0.0f});
}

void
status_led_driver::set_pattern(pattern_t pattern) {
    if (!good)
        return;
    std::optional<blending_t> blend = std::nullopt;
    if (this->current_pattern.pattern.blend_time != 0.0f) {
        auto last_color = this->current_pattern.sample();
        blend =
            blending_t(last_color, this->current_pattern.pattern.blend_time);
    }
    this->current_pattern = pattern_executor_t(pattern, blend);
}

void
status_led_driver::tick() {
    auto color = this->current_pattern.tick();
    if (!this->override_color) {
        good &= this->write_color(color);
    }
}

color_t
pattern_executor_t::sample() {
    auto t = this->timestamp;
    auto color = this->pattern.color;

    if (this->pattern.breathing_freq != 0.0f) {
        auto tf = (float)t * TICK_RATE;
        auto scale =
            (sin(2.0 * std::numbers::pi * tf * this->pattern.breathing_freq) +
             1.0) *
            0.5;
        color = color.scale(scale);
    }

    return color;
}

color_t
pattern_executor_t::tick() {
    auto color = this->sample();
    this->timestamp++;
    return color;
}
