#ifndef __INDX_STATUS_LED_DRIVER_H__
#define __INDX_STATUS_LED_DRIVER_H__

#include <optional>

#include "i2c_bus.h"

#pragma pack(push, 1)
struct color_t {
    constexpr color_t(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
    uint8_t r, g, b;

    color_t
    scale(float scale) {
        return color_t{(uint8_t)((float)r * scale), (uint8_t)((float)g * scale),
                       (uint8_t)((float)b * scale)};
    }

    static inline color_t
    blend(color_t a, color_t b, float b_ratio) {
        auto a_ratio = 1.0f - b_ratio;
        return color_t{
            (uint8_t)((float)b.r * b_ratio + (float)a.r * a_ratio),
            (uint8_t)((float)b.g * b_ratio + (float)a.g * a_ratio),
            (uint8_t)((float)b.b * b_ratio + (float)a.b * a_ratio),
        };
    }
};

struct pattern_t {
    color_t color;
    uint8_t blinks_fast;
    uint8_t blinks_slow;
    float breathing_freq;
    float blend_time;
};

; // namespace pattern_defs

#pragma pack(pop)

constexpr auto col_black = color_t(0, 0, 0);
constexpr auto col_red = color_t(255, 0, 0);
constexpr auto col_green = color_t(0, 255, 0);
constexpr auto col_blue = color_t(0, 0, 255);
constexpr auto col_amber = color_t(255, 64, 0);
constexpr auto col_half_amber = color_t(128, 32, 0);
constexpr auto col_purple = color_t(255, 0, 255);

constexpr auto pattern_off = pattern_t{col_black, 0, 0, 0.0f, 0.0f};
constexpr auto pattern_error_temp_min = pattern_t{col_red, 1, 1, 0.0f, 0.0f};
constexpr auto pattern_error_temp_max = pattern_t{col_red, 1, 2, 0.0f, 0.0f};
constexpr auto pattern_error_temp_ambient_min =
    pattern_t{col_red, 1, 3, 0.0f, 0.0f};
constexpr auto pattern_error_temp_ambient_max =
    pattern_t{col_red, 1, 4, 0.0f, 0.0f};
constexpr auto pattern_error_temp_diff = pattern_t{col_red, 1, 5, 0.0f, 0.0f};
constexpr auto pattern_error_heat_max = pattern_t{col_red, 2, 1, 0.0f, 0.0f};
constexpr auto pattern_error_heat_input = pattern_t{col_red, 2, 2, 0.0f, 0.0f};
constexpr auto pattern_error_internal_temp =
    pattern_t{col_red, 5, 3, 0.0f, 0.0f};
constexpr auto pattern_heating = pattern_t{col_amber, 0, 0, 0.5f, 0.0f};
constexpr auto pattern_cold = pattern_t{col_green, 0, 0, 0.0f, 0.5f};
constexpr auto pattern_target = pattern_t{col_amber, 0, 0, 0.0f, 0.5f};
constexpr auto pattern_cooling_down =
    pattern_t{col_half_amber, 0, 0, 0.0f, 0.5f};
constexpr auto pattern_cooled_down = pattern_t{col_blue, 0, 0, 0.0f, 0.5f};
constexpr auto pattern_dfu = pattern_t{col_blue, 1, 0, 0.0f, 0.0f};
constexpr auto pattern_transient = pattern_t{col_amber, 1, 0, 0.0f, 0.0f};

struct blending_t {
    blending_t(color_t initial_color, float time)
        : color(initial_color), time(time) {}

    inline color_t
    blend(float t, color_t new_color) {
        return color_t::blend(color, new_color, t / this->time);
    }

    inline bool
    is_finished(float t) {
        return t >= this->time;
    }

    color_t color;
    float time;
};

struct pattern_executor_t {
    pattern_executor_t(pattern_t pattern, std::optional<blending_t> blend)
        : pattern(pattern), blend(blend) {}

    pattern_t pattern;
    std::optional<blending_t> blend;
    uint32_t timestamp{0};

    color_t
    tick();

    color_t
    sample();
};

struct status_led_driver {
    status_led_driver(i2c_bus *bus);

    void
    set_currents(uint8_t red, uint8_t green, uint8_t blue);

    void
    force_color(std::optional<color_t> color);

    void
    set_color(color_t color);

    void
    set_pattern(pattern_t pattern);

    void
    tick();

  protected:
    bool
    write_u8(uint8_t reg, uint8_t val);

    bool
    write_color(color_t color);

    bool
    read_u8_into(uint8_t reg, uint8_t &out);

  private:
    i2c_bus *bus;
    bool good;
    pattern_executor_t current_pattern;
    std::optional<color_t> override_color;
};

#endif
