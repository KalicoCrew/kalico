extern "C" {
#include "atsamd/gpio.h"
#include "atsamd/internal.h"
#include "command.h"
#include "generic/irq.h"
#include "generic/misc.h"
#include "sched.h"
}

#include "coil_driver.h"

#define DRIVE_TCC TCC3
#define DRIVE_TCC_PCLK_ID TCC3_GCLK_ID
#define DRIVE_TCC_PM_ID ID_TCC3
// Channel for drive signal output
#define DRIVE_PIN GPIO('B', 17)
#define DRIVE_PIN_FUNC 'F'
#define DRIVE_TCC_CC 1
#define DRIVE_TCC_SYNCBUSY_CC TCC_SYNCBUSY_CC1
#define DRIVE_TCC_IRQn TCC3_0_IRQn
// Channel for ADC triggering during tuning
#define DRIVE_TCC_TUNE_CC 0
#define DRIVE_TCC_TUNE_SYNCBUSY_CC TCC_SYNCBUSY_CC0
#define DRIVE_TCC_TUNE_EVSYS_CHANNEL 2

// Length of burst modulation frame
static constexpr uint32_t FRAME_CYCLES = 512;

// Overvoltage and tuning input is via resistive divider
static constexpr float OV_DIVIDER_R1 = 45000.0f; // Drain to tap
static constexpr float OV_DIVIDER_R2 = 1500.0f; // Tap to gnd
// The feedback is on both pins PA06(for AC) and PB05(for ADC)
DECL_CONSTANT_STR("RESERVE_PINS_INDX_FEEDBACK_SENSE", "PA06,PB05");

// Feedback system
#define FEEDBACK_AC_PIN GPIO('A', 6)
#define FEEDBACK_AC_PIN_FUNC 'B'
#define FEEDBACK_ADC_PIN GPIO('B', 5)
#define FEEDBACK_ADC_PIN_FUNC 'B'

#define OV_EVSYS_CHANNEL 1
static constexpr float OV_VDDANA = 3.3f; // AC reference voltage
static constexpr float OV_TRIP_VOLTAGE = 100.0f; // overvoltage threshold
static constexpr float TUNE_TARGET_PEAK_V = 96.0f; // target drive peak voltage
//
static constexpr uint32_t OV_SCALER_VALUE =
    (uint32_t)(64.0f * OV_TRIP_VOLTAGE * OV_DIVIDER_R2 /
                   ((OV_DIVIDER_R1 + OV_DIVIDER_R2) * OV_VDDANA) +
               0.5f) -
    1;

static_assert(OV_SCALER_VALUE <= 63, "overvoltage scaler value out of range");

static constexpr uint32_t TUNE_TARGET_SCALER_VALUE =
    (uint32_t)(64.0f * TUNE_TARGET_PEAK_V * OV_DIVIDER_R2 /
                   ((OV_DIVIDER_R1 + OV_DIVIDER_R2) * OV_VDDANA) +
               0.5f) -
    1;
static_assert(TUNE_TARGET_SCALER_VALUE <= 63,
              "tune target scaler value out of range");
static_assert(TUNE_TARGET_SCALER_VALUE < OV_SCALER_VALUE,
              "Tuner target must be below overvoltage target");

// Feedback ADC for current sensing and sampling waveform during coil tuning
#define FEEDBACK_ADC ADC1
#define FEEDBACK_ADC_0_IRQn ADC1_0_IRQn
#define FEEDBACK_ADC_1_IRQn ADC1_1_IRQn
#define FEEDBACK_ADC_GCLK_ID ADC1_GCLK_ID
#define FEEDBACK_ADC_PM_ID ID_ADC1
#define DRAIN_SENSE_ADC_MUXPOS 7

// Current sense system definitions
#define CURRENT_SENSE_PIN GPIO('B', 6)
#define CURRENT_SENSE_PIN_FUNC 'B'
#define CURRENT_SENSE_ADC_MUXPOS 8
#define CURRENT_SENSE_RATE 32000
DECL_CONSTANT_STR("RESERVE_PINS_INDX_CURRENT_SENSE", "PB06");
// 3.3V / 4095 / (50 * 0.02 ohm)
DECL_CONSTANT_STR("INDX_CURRENT_SENSE_AMP_PER_COUNT", "0.000805860806");
DECL_CTR("DECL_CONSTANT INDX_CURRENT_SENSE_RATE 32000");

// Current sense pacing
#define CURRENT_SENSE_TRIGGER_TC TC2
#define CURRENT_SENSE_TRIGGER_TC_GCLK_ID TC2_GCLK_ID
#define CURRENT_SENSE_TRIGGER_TC_PM_ID ID_TC2
#define ADC_TRIGGER_EVSYS_CHANNEL 0
static constexpr uint32_t ADC_TRIGGER_PERIOD =
    CONFIG_CLOCK_FREQ / CURRENT_SENSE_RATE;

static constexpr float DRAIN_DIVIDER_GAIN =
    OV_DIVIDER_R2 / (OV_DIVIDER_R1 + OV_DIVIDER_R2);
static constexpr float ADC_COUNT_MAX = 4095.0f; // 12-bit full scale
static constexpr float DRAIN_V_PER_COUNT =
    OV_VDDANA / ADC_COUNT_MAX / DRAIN_DIVIDER_GAIN;

// ON-time search bounds (clock ticks).
static constexpr uint32_t TUNE_ON_FIRST_SEED = 180; // safe starting floor
static constexpr uint32_t TUNE_ON_STEP = 2; // ramp granularity
static constexpr uint32_t TUNE_ON_MAX = 1080;

// Provisional OFF used while ramping ON: long enough to contain a full positive
// resonance halfcycle even at the lowest resonant frequency.
static constexpr uint32_t TUNE_OFF_PROV = 840;

// OFF cycles fired after the last measured cycle so its resonance completes
// before the timer stops.
static constexpr uint32_t TUNE_TRAILING_OFF = 1;

// Quiet time between bursts, long enough for the tank to fully stop resonating.
static constexpr uint32_t TUNE_SETTLE_US = 350;

// Whole-tune watchdog. A full sweep is typically well under ~2 s, so this is
// just a timeout to ensure that session doesn't hang in case of hardware faults
// etc.
static constexpr uint32_t TUNE_TOTAL_TIMEOUT_US = 10000000; // 10 s

// Waveform sweep parameters for OFF time tuning. Must be long enough to
// encompass the full on+off cycle to ensure we can capture the full positive
// resonance halfcycle.
static constexpr uint32_t TUNE_SAMPLE_STEP = 8;
static constexpr uint32_t TUNE_N_SAMPLES =
    (TUNE_ON_MAX + TUNE_OFF_PROV) / TUNE_SAMPLE_STEP;

// Margin when finding the resonance cycle in the sampled data.
static constexpr float TUNE_ZERO_MARGIN_V = 1.0f;

// Waveform-dump pacing interval, to reduce risk of lost messages.
static constexpr uint32_t TUNE_DUMP_INTERVAL_US = 3000;

// Runtime state shared between task context and the TCC3/AC/ADC ISRs.
struct driver_state {
    // Compare value (ticks) for the steady ON portion.
    volatile uint32_t on_ticks;
    // Compare value for the first cycle ON portion in a burst.
    volatile uint32_t on_ticks_first;
    // Period-1 of a normal cycle, = on_ticks + off - 1.
    volatile uint32_t period_minus_1;
    // Period-1 of the first cycle in a burst, = on_ticks_first + off - 1
    volatile uint32_t first_period_minus_1;
    // ON cycles for the burst being scheduled (latched at the boundary).
    volatile uint32_t on_cycles;
    // ON cycles requested by set_duty(), latched into on_cycles each burst.
    volatile uint32_t pending_on_cycles;
    // Index (0..FRAME_CYCLES-1) of the next cycle the ISR will program.
    volatile uint32_t cycle_idx;
    // Whether the preceding cycle is ON. If not, we should use the first cycle
    // length instead.
    volatile bool prev_was_on;
    // Cumulative overvoltage trips, increased by the AC comparator ISR.
    volatile uint32_t ov_count;
    // Whether the drive timings above are valid to fire (set explicitly via
    // set_timings or by a successful tune).
    bool timings_valid;

    // (Re)start a stopped/faulted drive timer for the pending burst.
    void
    arm_timer();
    // Publish drive timings directly in ticks (ISR blocked across the writes).
    void
    set_drive_ticks(uint32_t on_ticks, uint32_t on_ticks_first,
                    uint32_t off_ticks);
    // Cumulative overvoltage trips since boot
    uint32_t
    overvoltage_count() {
        return __atomic_load_n(&ov_count, __ATOMIC_RELAXED);
    }

    // ISR hooks
    void
    schedule_next_cycle();

    void
    record_overvoltage() {
        // AC COMP0 overvoltage trip. The EVSYS channel already faulted the
        // timer, so we just record the event here.
        __atomic_fetch_add(&ov_count, 1, __ATOMIC_RELAXED);
    }
};

static driver_state state;

struct current_sense_state {
    uint32_t tally; // raw ADC counts, __atomic

    void
    on_sample(uint16_t result) {
        __atomic_fetch_add(&tally, result, __ATOMIC_RELAXED);
    }

    uint32_t
    total() {
        return __atomic_load_n(&tally, __ATOMIC_RELAXED);
    }
};

static current_sense_state current_sense;

enum class tune_phase : uint8_t {
    idle,
    on_first, // Phase 1: ramp first time_on until cycle 0's peak crosses target
    on_first_done, // start Phase 2
    off_measure, // Phase 2: equivalent-time sweep of the positive resonance
                 // half cycle
    off_analyze, // find time_off
    off_dump, // if requested, transfer the reconstructed waveform
              // (rate-limited)
    off_done, // start Phase 3
    steady_on, // Phase 3: ramp time_on until cycle 1's peak crosses target
    steady_done, // report all three timings
    error,
};

enum class tune_error : uint8_t {
    none = 0,
    overvoltage = 1, // Overvoltage triggered during the tune (target removed?)
    initial_on_ceiling =
        2, // ON-time tuning hit maximum time without reaching the target peak
    off_no_zero =
        3, // the resonance never returned to zero within the sweep window
    steady_ceiling = 4, // steady ON hit the ceiling without reaching the target
    timeout = 5, // timed out
};

enum class tune_status : uint8_t {
    idle = 0, // no tune has run since boot
    running = 1, // tuning in progress
    success = 2, // completed. on_first_ticks/off_ticks/on_ticks hold result
    failed = 3, // aborted. `error` holds the reason
};

enum class burst_status : uint8_t { busy, completed, ready };

// Bounded test burst used by coil tune and ringdown. Only one owner at a time.
struct test_burst {
    bool burst_in_flight{false};
    uint32_t next_fire_time{0};

    volatile bool test_active{false};
    volatile uint32_t cycles_done{0};
    volatile uint32_t stop_after{0};
    volatile uint32_t measure_cycle{0};
    volatile bool burst_done{false};
    volatile bool adc_active{false};
    volatile bool adc_capture_armed{false};
    volatile uint16_t adc_sample{0};

    bool
    on_drive_overflow();
    bool
    capture_adc_sample(uint16_t result);
    burst_status
    poll();
    void
    fire(uint32_t n_cycles, uint32_t measure);
    void
    adc_arm();
    void
    adc_restore();
    void
    stop_hw();
};

static test_burst burst;

struct tuner_state {
    bool want_details{false};

    tune_phase phase{tune_phase::idle};
    tune_error error{tune_error::none};
    // Latched across the return to idle so the host can poll the outcome.
    tune_status status{tune_status::idle};
    // Snapshot of the overvoltage count at start; any change aborts the tune.
    uint32_t ov_count_at_start;
    // Whole-tune watchdog deadline; passing it aborts the tune.
    uint32_t deadline;
    uint32_t on_first_ticks; // Phase 1 candidate, then the kept result
    uint32_t waveform_sample_count; // Desired number of waveform samples
    uint32_t waveform_sample_idx; // OFF-sweep / dump index
    uint32_t off_ticks; // result (ticks)
    uint32_t on_ticks; // Phase 3 candidate, then the kept result
    uint32_t found_steady; // result (ticks)

    bool
    active() {
        return phase != tune_phase::idle;
    }
    void
    start(bool want_details);
    void
    step();
    void
    abort(tune_error e);
    void
    finish(tune_status s);
    void
    step_on_first();
    void
    step_off_measure();
    void
    analyze_off();
    void
    step_steady_on();
};

static tuner_state tuner;

// --- Nozzle presence ringdown (first-lobe peak amplitude) ---

static constexpr uint32_t RINGDOWN_SAMPLE_STEP = 16;
static constexpr uint32_t RINGDOWN_N_SAMPLES = 64;
static constexpr uint32_t RINGDOWN_TIMEOUT_US = 2000000;
static constexpr float RINGDOWN_EXCITE_SCALE_DEFAULT = 0.8f;
// Soften the probe pulse (never hotter than calibrated coil_time_on_first).
// Default 0.8 from Bondtech characterisation (empty OV at 1.0).
static constexpr float RINGDOWN_EXCITE_SCALE_MIN = 0.05f;
static constexpr float RINGDOWN_EXCITE_SCALE_MAX = 1.0f;
// Peak-voltage thresholds from Bondtech EXCITE_SCALE=0.8 sweep
// (seated ~84 V, empty ~95 V). Re-characterise before heat_gate.
static constexpr float RINGDOWN_PRESENT_PEAK_V_DEFAULT = 87.0f;
static constexpr float RINGDOWN_ABSENT_PEAK_V_DEFAULT = 91.0f;
static constexpr float RINGDOWN_MIN_PEAK_V_DEFAULT = 20.0f;
static constexpr uint32_t RINGDOWN_IDLE_MS_DEFAULT = 500;
static constexpr uint32_t RINGDOWN_HEAT_MS_DEFAULT = 500;

enum class ringdown_phase : uint8_t {
    idle,
    pause_wait,
    sample,
    analyze,
    dump,
};

struct ringdown_config {
    bool enable{false};
    bool heat_gate{false};
    float present_peak_v{RINGDOWN_PRESENT_PEAK_V_DEFAULT};
    float absent_peak_v{RINGDOWN_ABSENT_PEAK_V_DEFAULT};
    float min_peak_v{RINGDOWN_MIN_PEAK_V_DEFAULT};
    float excite_scale{RINGDOWN_EXCITE_SCALE_DEFAULT};
    uint32_t idle_ms{RINGDOWN_IDLE_MS_DEFAULT};
    uint32_t heat_ms{RINGDOWN_HEAT_MS_DEFAULT};
};

struct ringdown_state {
    ringdown_phase phase{ringdown_phase::idle};
    ringdown_status status{ringdown_status::idle};
    nozzle_presence presence{nozzle_presence::unknown};
    bool want_report{false};
    bool want_dump{false};
    uint32_t ov_count_at_start{0};
    uint32_t deadline{0};
    uint32_t sample_idx{0};
    uint32_t dump_idx{0};
    uint32_t dump_count{0};
    uint32_t saved_pending_on_cycles{0};
    uint32_t saved_on_ticks{0};
    uint32_t saved_on_ticks_first{0};
    uint32_t saved_off_ticks{0};
    uint32_t peak_mv{0};
    ringdown_status pending_status{ringdown_status::idle};
    uint32_t next_schedule_time{0};

    bool
    active() {
        return phase != ringdown_phase::idle;
    }
    void
    start(bool report, bool dump);
    void
    step();
    void
    abort(ringdown_status s);
    void
    finish(ringdown_status s);
    void
    schedule_finish_or_dump(ringdown_status s);
    void
    apply_presence(ringdown_status s);
    void
    restore_drive();
    void
    analyze();
    uint32_t
    excite_on_ticks();
};

static ringdown_state ringdown;
static ringdown_config ringdown_cfg;
static nozzle_presence last_presence{nozzle_presence::unknown};
static uint16_t ringdown_samples[RINGDOWN_N_SAMPLES];

static bool
heat_gate_blocks_drive() {
    return ringdown_cfg.enable && ringdown_cfg.heat_gate &&
           last_presence != nozzle_presence::present;
}

// IRQ declarations and assert correct indices.

static_assert(FEEDBACK_ADC_1_IRQn == 121);
DECL_CTR("DECL_ARMCM_IRQ FEEDBACK_ADC_1_Handler 121");
static_assert(FEEDBACK_ADC_0_IRQn == 120);
DECL_CTR("DECL_ARMCM_IRQ FEEDBACK_ADC_0_Handler 120");

static_assert(DRIVE_TCC_IRQn == 101);
DECL_CTR("DECL_ARMCM_IRQ TCC3_0_Handler 101");
static_assert(AC_IRQn == 122);
DECL_CTR("DECL_ARMCM_IRQ AC_Handler 122");

// Start a drive burst from an idle tank state. Sets up first two cycles and
// starts the timer. Future cycles are set up by the timer ISR later on.
// Only called if the timer is idle, clears any latched overvoltage fault.
void
driver_state::arm_timer() {
    uint32_t on_cycles = pending_on_cycles;
    this->on_cycles = on_cycles;
    if (on_cycles == 0) {
        // Nothing to fire: leave the timer stopped, output low.
        return;
    }

    NVIC_DisableIRQ(DRIVE_TCC_IRQn);

    // A latched fault (STATUS.FAULT0) leaves the counter halted and WO1 low.
    // STOP then clear FAULT0. Clearing it while stopped keeps it stopped.
    if (DRIVE_TCC->STATUS.bit.FAULT0) {
        DRIVE_TCC->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;
        while (DRIVE_TCC->SYNCBUSY.bit.CTRLB)
            ;
        DRIVE_TCC->STATUS.reg = TCC_STATUS_FAULT0;
    }

    // Flush any PERBUF/CCBUF the ISR staged before the timer stopped (at a
    // frame boundary or a fault); it was never transferred, so PERBUFV/CCBUFV
    // stay set. A direct PER/CC write below would then never finish syncing (a
    // stopped counter has no UPDATE event to consume the buffer) and stall into
    // a watchdog reset. CMD_UPDATE clears them. No-op at first arm.
    DRIVE_TCC->CTRLBSET.reg = TCC_CTRLBSET_CMD_UPDATE;
    while (DRIVE_TCC->SYNCBUSY.bit.CTRLB)
        ;

    // Cycle 0 (inital ON ON + shortened period) goes in PER/CC. Cycle 1
    // (steady ON, or 0 if the burst is a single cycle) goes in
    // PERBUF/CCBUF, copied to PER/CC at the first overflow.
    bool on1 = on_cycles > 1;
    uint32_t cc1 = on1 ? on_ticks : 0;
    DRIVE_TCC->PER.reg = TCC_PER_PER(first_period_minus_1);
    DRIVE_TCC->CC[DRIVE_TCC_CC].reg = TCC_CC_CC(on_ticks_first);
    while (DRIVE_TCC->SYNCBUSY.reg & (TCC_SYNCBUSY_PER | DRIVE_TCC_SYNCBUSY_CC))
        ;
    DRIVE_TCC->PERBUF.reg = TCC_PERBUF_PERBUF(period_minus_1);
    DRIVE_TCC->CCBUF[DRIVE_TCC_CC].reg = TCC_CCBUF_CCBUF(cc1);

    cycle_idx = 2;
    prev_was_on = on1;

    // Unset stale overflow flag so the first ISR aligns with cycle 2.
    DRIVE_TCC->INTFLAG.reg = TCC_INTFLAG_OVF;

    // Zero COUNT before retriggering. A retrigger of a STOPPED counter resumes
    // from current COUNT. After a fault COUNT is frozen mid-cycle, so cycle 0
    // would otherwise start partway through.
    DRIVE_TCC->COUNT.reg = TCC_COUNT_COUNT(0);
    while (DRIVE_TCC->SYNCBUSY.bit.COUNT)
        ;

    DRIVE_TCC->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;
    while (DRIVE_TCC->SYNCBUSY.bit.CTRLB)
        ;

    NVIC_EnableIRQ(DRIVE_TCC_IRQn);
}

void
coil_driver::set_duty(float duty) {
    if (duty != 0.0f && tune_active()) {
        shutdown("INDX heater target set while coil tuning");
    }
    // The coil timings have no default. Driving without first setting them
    // (explicitly or via a successful tune) is a fault.
    if (duty != 0.0f && !state.timings_valid) {
        shutdown("INDX heater started before coil timings were set");
    }
    // Soft heat gate: mute drive unless a probe has classified present.
    if (duty != 0.0f && heat_gate_blocks_drive()) {
        duty = 0.0f;
    }

    uint32_t cycles;
    if (duty <= 0.0f) {
        cycles = 0;
    } else if (duty >= 1.0f) {
        cycles = FRAME_CYCLES;
    } else {
        cycles = (uint32_t)(duty * (float)FRAME_CYCLES);
    }
    if (this->duty_limit && cycles > this->duty_limit) {
        cycles = this->duty_limit;
    }
    // Probe owns the tank; latch the requested duty for restore. Do not zero
    // the request first or restore would drop heat until the next PID tick.
    if (ringdown_active()) {
        ringdown.saved_pending_on_cycles = cycles;
        return;
    }
    state.pending_on_cycles = cycles;

    // Rearm the timer if it's not currently active.
    bool idle = DRIVE_TCC->STATUS.bit.STOP || DRIVE_TCC->STATUS.bit.FAULT0;
    if (cycles > 0 && idle) {
        state.arm_timer();
    }
}

void
coil_driver::set_cycle_limit(uint32_t limit) {
    this->duty_limit = limit;
}

uint32_t
coil_driver::get_ov_count() {
    return state.overvoltage_count();
}

void
coil_driver::shutdown_() {
    // Disconnect timer from drive pin. The external gate pulldown holds the
    // MOSFET off.
    gpio_peripheral(DRIVE_PIN, 0, 0);
    DRIVE_TCC->CTRLA.bit.ENABLE = 0;
    NVIC_DisableIRQ(DRIVE_TCC_IRQn);

    // Disable the overvoltage comparator.
    AC->CTRLA.reg = 0;
    NVIC_DisableIRQ(AC_IRQn);

    // Stop ADC start events first so no further conversions begin.
    CURRENT_SENSE_TRIGGER_TC->COUNT16.CTRLA.bit.ENABLE = 0;
    NVIC_DisableIRQ(FEEDBACK_ADC_1_IRQn);
    NVIC_DisableIRQ(FEEDBACK_ADC_0_IRQn);
}

uint32_t
coil_driver::get_total_charge() {
    return current_sense.total();
}

// Per-cycle scheduler. Runs at each overflow and stages the next
// cycle's CCBUF/PERBUF, which are the timings for the cycle that will begin
// _after_ the one that just started.
void
driver_state::schedule_next_cycle() {
    uint32_t idx = cycle_idx;

    // Shape the next cycle. Default = OFF/skipped (CC=0, full period). The
    // burst's first fired cycle (prev OFF) is soft-start (shorter ON + period,
    // preserving OFF/ZVS time); other fired cycles are steady ON.
    bool on = idx < on_cycles;
    bool initial = on && !prev_was_on;
    uint32_t cc = 0;
    uint32_t per = period_minus_1;
    if (initial) {
        cc = on_ticks_first;
        per = first_period_minus_1;
    } else if (on) {
        cc = on_ticks;
    }
    DRIVE_TCC->CCBUF[DRIVE_TCC_CC].reg = TCC_CCBUF_CCBUF(cc);
    DRIVE_TCC->PERBUF.reg = TCC_PERBUF_PERBUF(per);
    prev_was_on = on;

    // Check if this is the final cycle of the burst. If so, advance to the next
    // burst.
    if (++idx >= FRAME_CYCLES) {
        idx = 0;
        on_cycles = pending_on_cycles;
        if (on_cycles == 0) {
            DRIVE_TCC->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;
        }
    }
    cycle_idx = idx;
}

extern "C" void
TCC3_0_Handler(void) {
    DRIVE_TCC->INTFLAG.reg = TCC_INTFLAG_OVF;
    if (burst.on_drive_overflow())
        return;
    state.schedule_next_cycle();
}

extern "C" void
AC_Handler(void) {
    AC->INTFLAG.reg = AC_INTFLAG_COMP0;
    state.record_overvoltage();
}

extern "C" void
FEEDBACK_ADC_1_Handler(void) {
    uint16_t result = FEEDBACK_ADC->RESULT.reg;
    if (burst.capture_adc_sample(result))
        return;
    current_sense.on_sample(result);
}

extern "C" void
FEEDBACK_ADC_0_Handler(void) {
    shutdown("coil driver feedback ADC overrun");
}

// Enables clock, running at core frequency instead of slower peripheral clkgen.
static void
enable_pclock_max(uint32_t pclk_id, int32_t pm_id) {
    auto val = GCLK_PCHCTRL_GEN(0) | GCLK_PCHCTRL_CHEN;
    GCLK->PCHCTRL[pclk_id].reg = val;
    while (GCLK->PCHCTRL[pclk_id].reg != val)
        ;
    uint32_t pm_port = pm_id / 32, pm_bit = 1 << (pm_id % 32);
    (&MCLK->APBAMASK.reg)[pm_port] |= pm_bit;
}

static void
setup_adc1() {
    // Set up the ADC temperature pin, which ensures `adc_init` gets called. We
    // let this load the calibration data etc for us.
    gpio_adc_setup(0xfe);

    gpio_peripheral(CURRENT_SENSE_PIN, CURRENT_SENSE_PIN_FUNC, 0);
    gpio_peripheral(FEEDBACK_ADC_PIN, FEEDBACK_ADC_PIN_FUNC, 0);

    enable_pclock(FEEDBACK_ADC_GCLK_ID, FEEDBACK_ADC_PM_ID);

    // Read calibration data so we have it after we reset the ADC
    auto calib = FEEDBACK_ADC->CALIB.reg;

    FEEDBACK_ADC->CTRLA.bit.SWRST = 1;
    while (FEEDBACK_ADC->SYNCBUSY.bit.SWRST)
        ;

    // Reload calibration
    FEEDBACK_ADC->CALIB.reg = calib;

    FEEDBACK_ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;
    while (FEEDBACK_ADC->SYNCBUSY.bit.REFCTRL)
        ;

    FEEDBACK_ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(4);
    while (FEEDBACK_ADC->SYNCBUSY.bit.SAMPCTRL)
        ;

    // Start out with current sense as our ADC channel
    FEEDBACK_ADC->INPUTCTRL.reg =
        ADC_INPUTCTRL_MUXPOS(CURRENT_SENSE_ADC_MUXPOS) |
        ADC_INPUTCTRL_MUXNEG(0x18);
    while (FEEDBACK_ADC->SYNCBUSY.bit.INPUTCTRL)
        ;

    // ADC runs event triggered. CURRENT_SENSE_TRIGGER_TC paces current sense
    // measurements via EVSYS. During tuning, DRIVE_TCC takes over.
    FEEDBACK_ADC->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT;
    while (FEEDBACK_ADC->SYNCBUSY.bit.CTRLB)
        ;
    FEEDBACK_ADC->EVCTRL.reg = ADC_EVCTRL_STARTEI;

    FEEDBACK_ADC->INTENSET.reg = ADC_INTENSET_RESRDY | ADC_INTENSET_OVERRUN;

    NVIC_SetPriority(FEEDBACK_ADC_1_IRQn, 1);
    NVIC_SetPriority(FEEDBACK_ADC_0_IRQn, 1);
    NVIC_EnableIRQ(FEEDBACK_ADC_1_IRQn);
    NVIC_EnableIRQ(FEEDBACK_ADC_0_IRQn);

    // 48 MHz / 4 = 12 MHz ADC clock, spec calls for < 16MHz.
    FEEDBACK_ADC->CTRLA.reg =
        ADC_CTRLA_PRESCALER(ADC_CTRLA_PRESCALER_DIV4_Val) | ADC_CTRLA_ENABLE;
    while (FEEDBACK_ADC->SYNCBUSY.bit.ENABLE)
        ;
}

static void
setup_adc_trigger_tc() {
    enable_pclock_max(CURRENT_SENSE_TRIGGER_TC_GCLK_ID,
                      CURRENT_SENSE_TRIGGER_TC_PM_ID);

    CURRENT_SENSE_TRIGGER_TC->COUNT16.CTRLA.bit.SWRST = 1;
    while (CURRENT_SENSE_TRIGGER_TC->COUNT16.SYNCBUSY.bit.SWRST)
        ;

    CURRENT_SENSE_TRIGGER_TC->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;
    CURRENT_SENSE_TRIGGER_TC->COUNT16.CC[0].reg = ADC_TRIGGER_PERIOD - 1;
    CURRENT_SENSE_TRIGGER_TC->COUNT16.EVCTRL.reg = TC_EVCTRL_OVFEO;
    CURRENT_SENSE_TRIGGER_TC->COUNT16.CTRLA.reg =
        TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
    while (CURRENT_SENSE_TRIGGER_TC->COUNT16.SYNCBUSY.bit.ENABLE)
        ;
}

static void
setup_evsys() {
    // We need EVSYS for signal routing so bring it up now.
    MCLK->APBBMASK.bit.EVSYS_ = 1;

    // Event channel for triggering ADC for tuning and current sensing
    EVSYS->Channel[DRIVE_TCC_TUNE_EVSYS_CHANNEL].CHANNEL.reg =
        EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TCC3_MC_0) |
        EVSYS_CHANNEL_PATH_ASYNCHRONOUS;

    EVSYS->Channel[ADC_TRIGGER_EVSYS_CHANNEL].CHANNEL.reg =
        EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TC2_OVF) |
        EVSYS_CHANNEL_PATH_ASYNCHRONOUS;
    EVSYS->USER[EVSYS_ID_USER_ADC1_START].reg = ADC_TRIGGER_EVSYS_CHANNEL + 1;
}

static void
setup_drive_tcc() {
    enable_pclock_max(DRIVE_TCC_PCLK_ID, DRIVE_TCC_PM_ID);

    // Set up the drive timer.
    DRIVE_TCC->CTRLA.bit.SWRST = 1;
    while (DRIVE_TCC->SYNCBUSY.bit.SWRST)
        ;

    DRIVE_TCC->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1;
    DRIVE_TCC->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
    // On a non-recoverable fault, force WO1 (PB17) low so the MOSFET turns off.
    DRIVE_TCC->DRVCTRL.reg = TCC_DRVCTRL_NRE(1 << DRIVE_TCC_CC);
    // Overvoltage triggeres a non-recoverable fault on the drive timer via
    // EVSYS on event 0. During tuning, CC0 match outputs an event to trigger
    // ADC sampling.
    DRIVE_TCC->EVCTRL.reg =
        TCC_EVCTRL_TCEI0 | TCC_EVCTRL_EVACT0_FAULT | TCC_EVCTRL_MCEO0;

    DRIVE_TCC->CTRLA.bit.ENABLE = 1;
    while (DRIVE_TCC->SYNCBUSY.bit.ENABLE)
        ;
    DRIVE_TCC->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;
    while (DRIVE_TCC->SYNCBUSY.bit.CTRLB)
        ;

    DRIVE_TCC->INTENSET.reg = TCC_INTENSET_OVF;
    NVIC_SetPriority(DRIVE_TCC_IRQn, 1);
    NVIC_EnableIRQ(DRIVE_TCC_IRQn);

    gpio_peripheral(DRIVE_PIN, DRIVE_PIN_FUNC, false);
}

static void
setup_ac() {
    gpio_peripheral(FEEDBACK_AC_PIN, FEEDBACK_AC_PIN_FUNC, 0);

    enable_pclock(AC_GCLK_ID, ID_AC);

    AC->CTRLA.bit.SWRST = 1;
    while (AC->SYNCBUSY.bit.SWRST)
        ;

    // COMP0: overvoltage detector, interrupt triggers on rising edge which
    // indicates over voltage.
    AC->SCALER[0].reg = AC_SCALER_VALUE(OV_SCALER_VALUE);
    AC->COMPCTRL[0].reg = AC_COMPCTRL_MUXPOS_PIN2 | AC_COMPCTRL_MUXNEG_VSCALE |
                          AC_COMPCTRL_SPEED_HIGH | AC_COMPCTRL_INTSEL_RISING |
                          AC_COMPCTRL_HYSTEN | AC_COMPCTRL_HYST_HYST50 |
                          AC_COMPCTRL_ENABLE;
    while (AC->SYNCBUSY.bit.COMPCTRL0)
        ;

    // COMP1: tuner peak detector. Latches rising edge, not used in normal
    // driving. No hysteresis as that would bias the detect peak high.
    AC->SCALER[1].reg = AC_SCALER_VALUE(TUNE_TARGET_SCALER_VALUE);
    AC->COMPCTRL[1].reg = AC_COMPCTRL_MUXPOS_PIN2 | AC_COMPCTRL_MUXNEG_VSCALE |
                          AC_COMPCTRL_SPEED_HIGH | AC_COMPCTRL_INTSEL_RISING |
                          AC_COMPCTRL_ENABLE;
    while (AC->SYNCBUSY.bit.COMPCTRL1)
        ;

    AC->EVCTRL.reg = AC_EVCTRL_COMPEO0; // COMP0 state sent to event output
    AC->INTENSET.reg =
        AC_INTENSET_COMP0; // COMP0 rising edge triggers AC_Handler

    NVIC_SetPriority(AC_IRQn, 2);
    NVIC_EnableIRQ(AC_IRQn);

    AC->CTRLA.reg = AC_CTRLA_ENABLE;
    while (AC->SYNCBUSY.bit.ENABLE)
        ;

    // COMP0 is routed to event on DRIVE_TCC/TCC3 so next cycle can be
    // prevented.
    EVSYS->Channel[OV_EVSYS_CHANNEL].CHANNEL.reg =
        EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_AC_COMP_0) |
        EVSYS_CHANNEL_PATH_ASYNCHRONOUS;
    EVSYS->USER[EVSYS_ID_USER_TCC3_EV_0].reg = OV_EVSYS_CHANNEL + 1;
}

void
coil_driver::set_timings(float time_on, float time_off, float time_on_first) {
    auto osc_clk_freq = (float)CONFIG_CLOCK_FREQ;

    uint32_t on_ticks = (uint32_t)(osc_clk_freq * time_on);
    uint32_t off_ticks = (uint32_t)(osc_clk_freq * time_off);
    uint32_t on_ticks_first = (uint32_t)(osc_clk_freq * time_on_first);
    state.set_drive_ticks(on_ticks, on_ticks_first, off_ticks);
}

coil_driver
coil_driver_create() {
    auto driver = coil_driver();

    setup_evsys();
    setup_drive_tcc();
    setup_ac();
    setup_adc1();
    setup_adc_trigger_tc();

    return driver;
}

static constexpr uint32_t
volts_to_count(float v) {
    int32_t c = (int32_t)(v / DRAIN_V_PER_COUNT + 0.5f);
    if (c < 0)
        c = 0;
    if (c > 4095)
        c = 4095;
    return (uint32_t)c;
}

static uint32_t
ticks_to_ns(uint32_t ticks) {
    return (uint32_t)((float)ticks * (1.0e9f / (float)CONFIG_CLOCK_FREQ) +
                      0.5f);
}

// Read and clear the latched COMP1 peak flag.
static bool
tune_peak_tripped() {
    if (!AC->INTFLAG.bit.COMP1)
        return false;
    AC->INTFLAG.reg = AC_INTFLAG_COMP1;
    return true;
}

// Set new drive timings in ticks. If the specified timings are not valid,
// `timings_valid` is set to false and the currently set ticks are not touched.
void
driver_state::set_drive_ticks(uint32_t on_ticks, uint32_t on_ticks_first,
                              uint32_t off_ticks) {
    if (on_ticks == 0 || off_ticks == 0 || on_ticks_first == 0 ||
        on_ticks_first > on_ticks) {
        this->timings_valid = false;
        return;
    }

    NVIC_DisableIRQ(DRIVE_TCC_IRQn);
    this->on_ticks = on_ticks;
    this->period_minus_1 = on_ticks + off_ticks - 1;
    this->on_ticks_first = on_ticks_first;
    this->first_period_minus_1 = on_ticks_first + off_ticks - 1;
    this->timings_valid = true;
    NVIC_EnableIRQ(DRIVE_TCC_IRQn);
}

// Fire a single test burst from idle timer. Can fire either 1 or 2 cycles.
void
test_burst::fire(uint32_t n_cycles, uint32_t measure) {
    cycles_done = 0;
    stop_after = n_cycles + TUNE_TRAILING_OFF;
    measure_cycle = measure;
    burst_done = false;
    test_active = true;
    state.pending_on_cycles = n_cycles;
    burst_in_flight = true;
    state.arm_timer();
}

bool
test_burst::on_drive_overflow() {
    if (!test_active)
        return false;

    uint32_t started = cycles_done + 1;
    cycles_done = started;

    if (started == measure_cycle) {
        AC->INTFLAG.reg = AC_INTFLAG_COMP1;
    }

    DRIVE_TCC->CCBUF[DRIVE_TCC_CC].reg = TCC_CCBUF_CCBUF(0);
    DRIVE_TCC->PERBUF.reg = TCC_PERBUF_PERBUF(state.period_minus_1);

    if (started >= stop_after) {
        DRIVE_TCC->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;
        test_active = false;
        burst_done = true;
    }
    return true;
}

bool
test_burst::capture_adc_sample(uint16_t result) {
    if (!adc_active)
        return false;
    if (adc_capture_armed) {
        adc_sample = result;
        adc_capture_armed = false;
    }
    return true;
}

static void
tune_set_cc0(uint32_t value) {
    DRIVE_TCC->CC[DRIVE_TCC_TUNE_CC].reg = TCC_CC_CC(value);
    while (DRIVE_TCC->SYNCBUSY.reg & DRIVE_TCC_TUNE_SYNCBUSY_CC)
        ;
}

void
test_burst::adc_arm() {
    FEEDBACK_ADC->INPUTCTRL.reg =
            ADC_INPUTCTRL_MUXPOS(DRAIN_SENSE_ADC_MUXPOS) | ADC_INPUTCTRL_MUXNEG(0x18);
    while (FEEDBACK_ADC->SYNCBUSY.bit.INPUTCTRL)
        ;
    adc_active = true;
    EVSYS->USER[EVSYS_ID_USER_ADC1_START].reg =
        DRIVE_TCC_TUNE_EVSYS_CHANNEL + 1;
}

void
test_burst::adc_restore() {
    FEEDBACK_ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS(CURRENT_SENSE_ADC_MUXPOS) |
        ADC_INPUTCTRL_MUXNEG(0x18);
    while (FEEDBACK_ADC->SYNCBUSY.bit.INPUTCTRL)
        ;
    adc_active = false;
    EVSYS->USER[EVSYS_ID_USER_ADC1_START].reg = ADC_TRIGGER_EVSYS_CHANNEL + 1;
}

void
test_burst::stop_hw() {
    if (adc_active)
        adc_restore();
    DRIVE_TCC->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;
    test_active = false;
    burst_in_flight = false;
}

static uint16_t tune_waveform_samples[TUNE_N_SAMPLES];

burst_status
test_burst::poll() {
    if (burst_in_flight) {
        if (!burst_done)
            return burst_status::busy;
        burst_in_flight = false;
        next_fire_time = timer_read_time() + timer_from_us(TUNE_SETTLE_US);
        return burst_status::completed;
    }
    if (timer_is_before(timer_read_time(), next_fire_time))
        return burst_status::busy;
    return burst_status::ready;
}

void
tuner_state::start(bool want_details) {
    // start_tune() has already verified the coil is idle.
    *this = tuner_state{};
    this->want_details = want_details;
    phase = tune_phase::on_first;
    status = tune_status::running;
    on_first_ticks = TUNE_ON_FIRST_SEED;
    burst.next_fire_time = timer_read_time();
    ov_count_at_start = state.overvoltage_count();
    deadline = timer_read_time() + timer_from_us(TUNE_TOTAL_TIMEOUT_US);
    // The test bursts overwrite the published drive timings, so any previously
    // valid set is now gone; only a successful tune (or a fresh set_timings)
    // makes them valid again.
    state.timings_valid = false;
}

void
tuner_state::abort(tune_error e) {
    burst.stop_hw();
    // A failed tune has no valid result; clear the timings so a poll can't read
    // a stale/partial value (`error` carries the reason instead).
    on_first_ticks = 0;
    off_ticks = 0;
    error = e;
    phase = tune_phase::error;
}

void
tuner_state::finish(tune_status s) {
    status = s;
    // A successful tune leaves the found timings in the drive state,
    // so heating may use them
    if (s == tune_status::success) {
        state.timings_valid = true;
    }
    sendf("indx_coil_tune_finished status=%c error=%c", (uint32_t)s,
          (uint32_t)error);
    phase = tune_phase::idle;
}

static constexpr uint32_t
counts_to_mv(uint32_t counts) {
    return (uint32_t)((float)counts * DRAIN_V_PER_COUNT * 1000.0f + 0.5f);
}

// Phase 1: from a de-energized tank, fire single-cycle bursts at increasing
// time_on_first until the resonance peak first crosses the target.
void
tuner_state::step_on_first() {
    switch (burst.poll()) {
    case burst_status::busy:
        return;
    case burst_status::completed:
        if (tune_peak_tripped()) {
            // reached the target peak
            phase = tune_phase::on_first_done;
            return;
        }
        on_first_ticks += TUNE_ON_STEP;
        if (on_first_ticks > TUNE_ON_MAX) {
            abort(tune_error::initial_on_ceiling);
        }
        return;
    case burst_status::ready:
        // Single-cycle burst: cycle is the only fired cycle, so any peak is
        // unambiguously its own. Steady ON time is irrelevant here.
        state.set_drive_ticks(on_first_ticks, on_first_ticks, TUNE_OFF_PROV);
        tune_peak_tripped(); // clear target peak latch
        burst.fire(1, 0);
        return;
    }
}

// Phase 2: with fixed initial cycle ON time, run consecutive bursts and do
// equivalent-time sampling of the resonance waveform. To work around timing
// issues around firing the sampler, we grab the entire waveform before the
// actual OFF event as well. This way, when we go to find the actual OFF time we
// can measure between the zero points instead of relying on the start of the
// waveform being at exactly the OFF phase start.
void
tuner_state::step_off_measure() {
    switch (burst.poll()) {
    case burst_status::busy:
        return;
    case burst_status::completed:
        tune_waveform_samples[waveform_sample_idx] = burst.adc_sample;
        if (++waveform_sample_idx >= waveform_sample_count) {
            burst.adc_restore();
            phase = tune_phase::off_analyze;
        }
        return;
    case burst_status::ready:
        // Excite with the found time_on_first and step the CC0 match further
        // into cycle 0's OFF/resonance each burst.
        state.set_drive_ticks(on_first_ticks, on_first_ticks, TUNE_OFF_PROV);
        tune_set_cc0(waveform_sample_idx * TUNE_SAMPLE_STEP);
        burst.adc_capture_armed = true;
        burst.fire(1, 0);
        return;
    }
}

// Analyze captured waveform to find ideal off time.
// The sampled waveform right now contains (most of) the ON phase, and the
// entire OFF phase. We need to determine the total duration of the OFF phase.
void
tuner_state::analyze_off() {
    auto zero_threshold = volts_to_count(TUNE_ZERO_MARGIN_V);

    // Find the start and end of the OFF phase. Note that the signal rise/fall
    // is so fast that we don't really need to deal with hysteresis when finding
    // the edges.
    int16_t phase_off_start = -1;
    int16_t phase_off_end = -1;
    for (uint32_t i = 0; i < waveform_sample_count; i++) {
        auto sample = tune_waveform_samples[i];
        if (phase_off_start < 0 && sample >= zero_threshold) {
            // ON->OFF transition
            phase_off_start = i;
        } else if (phase_off_start >= 0 && phase_off_end < 0 &&
                   sample <= zero_threshold) {
            // OFF->ON transition
            phase_off_end = i;
            break;
        }
    }

    // If we didn't find both start and end, we can't find the OFF time.
    if (phase_off_start == -1 || phase_off_end == -1) {
        abort(tune_error::off_no_zero);
        return;
    }

    off_ticks = (phase_off_end - phase_off_start + 1) * TUNE_SAMPLE_STEP;

    // If details were requested, enter dump phase.
    burst.next_fire_time =
        timer_read_time() + (want_details ? 0 : timer_from_us(TUNE_SETTLE_US));
    phase = want_details ? tune_phase::off_dump : tune_phase::off_done;
    output("C %u", want_details);
    waveform_sample_idx = 0;
}

// Phase 3: find the steady-state ON time, that is the ON time for any phase
// after the first one. Ramp the steady state ON time from the time found for
// the initial ON time until the target is crossed. The starting point was set
// by the `off_done` phase handler.
void
tuner_state::step_steady_on() {
    switch (burst.poll()) {
    case burst_status::busy:
        return;
    case burst_status::completed:
        if (tune_peak_tripped()) {
            // target peak reached
            phase = tune_phase::steady_done;
            return;
        }
        on_ticks += TUNE_ON_STEP;
        if (on_ticks > TUNE_ON_MAX) {
            abort(tune_error::steady_ceiling);
        }
        return;
    case burst_status::ready:
        state.set_drive_ticks(on_ticks, on_first_ticks, off_ticks);
        tune_peak_tripped(); // clear target peak latch
        burst.fire(2, 1);
        return;
    }
}

void
tuner_state::step() {
    // Guards shared for all tuning phases.
    if (phase != tune_phase::idle && phase != tune_phase::error) {
        if (state.overvoltage_count() != ov_count_at_start) {
            abort(tune_error::overvoltage);
        } else if (!timer_is_before(timer_read_time(), deadline)) {
            abort(tune_error::timeout);
        }
    }

    switch (phase) {
    case tune_phase::on_first:
        step_on_first();
        break;

    case tune_phase::on_first_done:
        burst.adc_arm(); // Remap ADC to feedback input for off sweep phase
        waveform_sample_idx = 0;
        waveform_sample_count =
            (on_first_ticks + TUNE_OFF_PROV) / TUNE_SAMPLE_STEP;
        phase = tune_phase::off_measure;
        break;

    case tune_phase::off_measure:
        step_off_measure();
        break;

    case tune_phase::off_analyze:
        analyze_off();
        break;

    case tune_phase::off_dump:
        // Stream the reconstructed waveform one sample at a time, so
        // it can be overlaid on the scope trace to validate the measurement.
        // Since this is diagnostic only, dropped samples are acceptable.
        if (timer_is_before(timer_read_time(), burst.next_fire_time))
            break;
        if (waveform_sample_idx < waveform_sample_count) {
            sendf("indx_coil_tune_wave index=%u ns=%u mv=%u",
                  waveform_sample_idx,
                  ticks_to_ns(waveform_sample_idx * TUNE_SAMPLE_STEP),
                  counts_to_mv(tune_waveform_samples[waveform_sample_idx]));
            waveform_sample_idx++;
            burst.next_fire_time =
                timer_read_time() + timer_from_us(TUNE_DUMP_INTERVAL_US);
        } else {
            burst.next_fire_time = timer_read_time();
            phase = tune_phase::off_done;
        }
        break;

    case tune_phase::off_done:
        // Phase 3: steady state ON time is always >= the initial ON time, so
        // start from there. Next fire time was set by the previous phase.
        on_ticks = on_first_ticks;
        phase = tune_phase::steady_on;
        break;

    case tune_phase::steady_on:
        step_steady_on();
        break;

    case tune_phase::steady_done:
        finish(tune_status::success);
        break;

    case tune_phase::error:
        finish(tune_status::failed);
        break;

    case tune_phase::idle:
        break;
    }
}

void
coil_driver::start_tune(bool want_details) {
    if (tuner.active() || ringdown.active()) {
        return;
    }
    if (state.pending_on_cycles != 0) {
        shutdown("INDX coil tune requested while heater active");
    }
    tuner.start(want_details);
}

bool
coil_driver::tune_active() {
    return tuner.active();
}

void
coil_driver::tune_step() {
    tuner.step();
}

void
coil_driver::report_tune_status() {
    sendf("indx_coil_tune_result status=%c error=%c on_first=%u off=%u on=%u",
          (uint32_t)tuner.status, (uint32_t)tuner.error,
          ticks_to_ns(tuner.on_first_ticks), ticks_to_ns(tuner.off_ticks),
          ticks_to_ns(tuner.on_ticks));
}

void
coil_driver::report_params() {
    uint32_t on = state.on_ticks;
    uint32_t off = state.period_minus_1 + 1 - on;
    sendf("indx_coil_driver_params time_on=%u time_off=%u time_on_first=%u",
          ticks_to_ns(on), ticks_to_ns(off), ticks_to_ns(state.on_ticks_first));
}

// ---------------------------------------------------------------------------
// Ringdown nozzle-presence probe
// ---------------------------------------------------------------------------

void
ringdown_state::apply_presence(ringdown_status s) {
    if (s == ringdown_status::valid) {
        last_presence = presence;
    } else if (s != ringdown_status::running) {
        if (presence == nozzle_presence::present)
            presence = nozzle_presence::unknown;
        last_presence = presence;
    }
}

void
ringdown_state::restore_drive() {
    if (burst.adc_active)
        burst.adc_restore();
    state.set_drive_ticks(saved_on_ticks, saved_on_ticks_first, saved_off_ticks);
    uint32_t cycles = saved_pending_on_cycles;
    if (heat_gate_blocks_drive())
        cycles = 0;
    state.pending_on_cycles = cycles;
    bool idle = DRIVE_TCC->STATUS.bit.STOP || DRIVE_TCC->STATUS.bit.FAULT0;
    if (cycles > 0 && idle) {
        state.arm_timer();
    }
}

void
ringdown_state::abort(ringdown_status s) {
    burst.test_active = false;
    burst.burst_in_flight = false;
    presence = nozzle_presence::unknown;
    apply_presence(s);
    restore_drive();
    if (phase == ringdown_phase::dump) {
        finish(s);
        return;
    }
    dump_count =
        sample_idx < RINGDOWN_N_SAMPLES ? sample_idx : RINGDOWN_N_SAMPLES;
    schedule_finish_or_dump(s);
}

void
ringdown_state::schedule_finish_or_dump(ringdown_status s) {
    pending_status = s;
    if (want_dump) {
        dump_idx = 0;
        burst.next_fire_time = timer_read_time();
        phase = ringdown_phase::dump;
        return;
    }
    finish(s);
}

void
ringdown_state::finish(ringdown_status s) {
    status = s;
    apply_presence(s);
    if (want_report) {
        sendf("indx_nozzle_presence clock=%u status=%c presence=%c peak_mv=%u",
              timer_read_time(), (uint32_t)status, (uint32_t)presence, peak_mv);
    }
    next_schedule_time = timer_read_time();
    phase = ringdown_phase::idle;
}

uint32_t
ringdown_state::excite_on_ticks() {
    float scale = ringdown_cfg.excite_scale;
    if (scale < RINGDOWN_EXCITE_SCALE_MIN)
        scale = RINGDOWN_EXCITE_SCALE_MIN;
    if (scale > RINGDOWN_EXCITE_SCALE_MAX)
        scale = RINGDOWN_EXCITE_SCALE_MAX;
    uint32_t ticks = (uint32_t)(saved_on_ticks_first * scale + 0.5f);
    if (ticks < 1)
        ticks = 1;
    if (saved_on_ticks_first && ticks > saved_on_ticks_first)
        ticks = saved_on_ticks_first;
    return ticks;
}

void
ringdown_state::start(bool report, bool dump) {
    phase = ringdown_phase::pause_wait;
    status = ringdown_status::running;
    presence = nozzle_presence::unknown;
    want_report = report;
    want_dump = dump;
    burst.burst_in_flight = false;
    burst.test_active = false;
    burst.burst_done = false;
    burst.adc_capture_armed = false;
    burst.adc_sample = 0;
    burst.next_fire_time = timer_read_time() + timer_from_us(TUNE_SETTLE_US);
    ov_count_at_start = state.overvoltage_count();
    uint32_t timeout_us = RINGDOWN_TIMEOUT_US;
    if (dump)
        timeout_us += RINGDOWN_N_SAMPLES * TUNE_DUMP_INTERVAL_US + 50000;
    deadline = timer_read_time() + timer_from_us(timeout_us);
    sample_idx = 0;
    dump_idx = 0;
    dump_count = 0;
    peak_mv = 0;
    pending_status = ringdown_status::idle;
    saved_pending_on_cycles = state.pending_on_cycles;
    saved_on_ticks = state.on_ticks;
    saved_on_ticks_first = state.on_ticks_first;
    saved_off_ticks = state.period_minus_1 + 1 - state.on_ticks;
    state.pending_on_cycles = 0;
}

void
ringdown_state::analyze() {
    dump_count = RINGDOWN_N_SAMPLES;
    uint16_t max_counts = 0;
    for (uint32_t i = 0; i < RINGDOWN_N_SAMPLES; i++) {
        if (ringdown_samples[i] > max_counts)
            max_counts = ringdown_samples[i];
    }
    peak_mv = counts_to_mv(max_counts);
    float peak_v = (float)peak_mv / 1000.0f;

    if (peak_v < ringdown_cfg.min_peak_v) {
        presence = nozzle_presence::unknown;
        apply_presence(ringdown_status::weak_signal);
        restore_drive();
        schedule_finish_or_dump(ringdown_status::weak_signal);
        return;
    }

    // Seated loads the coil: lower peak. Empty: higher peak.
    if (peak_v <= ringdown_cfg.present_peak_v) {
        presence = nozzle_presence::present;
    } else if (peak_v >= ringdown_cfg.absent_peak_v) {
        presence = nozzle_presence::absent;
    } else {
        presence = nozzle_presence::unknown;
    }

    apply_presence(ringdown_status::valid);
    restore_drive();
    schedule_finish_or_dump(ringdown_status::valid);
}

void
ringdown_state::step() {
    if (phase == ringdown_phase::idle)
        return;

    if (state.overvoltage_count() != ov_count_at_start) {
        abort(ringdown_status::overvoltage);
        return;
    }
    if (!timer_is_before(timer_read_time(), deadline)) {
        abort(ringdown_status::timeout);
        return;
    }

    uint32_t on_ticks = excite_on_ticks();
    switch (phase) {
    case ringdown_phase::pause_wait: {
        bool idle = DRIVE_TCC->STATUS.bit.STOP || DRIVE_TCC->STATUS.bit.FAULT0;
        if (!idle || timer_is_before(timer_read_time(), burst.next_fire_time))
            return;
        state.set_drive_ticks(on_ticks, on_ticks, saved_off_ticks);
        burst.adc_arm();
        sample_idx = 0;
        phase = ringdown_phase::sample;
        return;
    }
    case ringdown_phase::sample: {
        switch (burst.poll()) {
        case burst_status::busy:
            return;
        case burst_status::completed:
            ringdown_samples[sample_idx] = burst.adc_sample;
            if (++sample_idx >= RINGDOWN_N_SAMPLES) {
                burst.adc_restore();
                phase = ringdown_phase::analyze;
            }
            return;
        case burst_status::ready:
            state.set_drive_ticks(on_ticks, on_ticks, saved_off_ticks);
            tune_set_cc0(sample_idx * RINGDOWN_SAMPLE_STEP);
            burst.adc_capture_armed = true;
            burst.fire(1, 0);
            return;
        }
        return;
    }
    case ringdown_phase::analyze:
        analyze();
        return;
    case ringdown_phase::dump: {
        if (timer_is_before(timer_read_time(), burst.next_fire_time))
            return;
        if (dump_idx < dump_count) {
            sendf("indx_ringdown_wave index=%u ns=%u counts=%u mv=%u", dump_idx,
                        ticks_to_ns(dump_idx * RINGDOWN_SAMPLE_STEP),
                        (uint32_t)ringdown_samples[dump_idx],
                        counts_to_mv(ringdown_samples[dump_idx]));
            dump_idx++;
            burst.next_fire_time =
                    timer_read_time() + timer_from_us(TUNE_DUMP_INTERVAL_US);
            return;
        }
        sendf("indx_ringdown_meta status=%c peak_mv=%u", (uint32_t)pending_status,
                    peak_mv);
        finish(pending_status);
        return;
    }
    case ringdown_phase::idle:
        break;
    }
}

void coil_driver::set_ringdown_params(bool enable, bool heat_gate,
                                                                            float present_peak_v, float absent_peak_v,
                                                                            uint32_t idle_ms, uint32_t heat_ms,
                                                                            float excite_scale, float min_peak_v) {
    if (excite_scale < RINGDOWN_EXCITE_SCALE_MIN)
        excite_scale = RINGDOWN_EXCITE_SCALE_MIN;
    if (excite_scale > RINGDOWN_EXCITE_SCALE_MAX)
        excite_scale = RINGDOWN_EXCITE_SCALE_MAX;
    ringdown_cfg.enable = enable;
    ringdown_cfg.heat_gate = heat_gate;
    ringdown_cfg.idle_ms = idle_ms ? idle_ms : RINGDOWN_IDLE_MS_DEFAULT;
    ringdown_cfg.heat_ms = heat_ms ? heat_ms : RINGDOWN_HEAT_MS_DEFAULT;
    ringdown_cfg.excite_scale = excite_scale;
    if (min_peak_v < 0.0f)
        min_peak_v = RINGDOWN_MIN_PEAK_V_DEFAULT;
    ringdown_cfg.min_peak_v =
            min_peak_v > 0.0f ? min_peak_v : RINGDOWN_MIN_PEAK_V_DEFAULT;
    if (present_peak_v < absent_peak_v) {
        ringdown_cfg.present_peak_v = present_peak_v;
        ringdown_cfg.absent_peak_v = absent_peak_v;
    }
}

void
coil_driver::start_ringdown(bool report, bool dump) {
    if (ringdown.active() || tuner.active())
        return;
    if (!state.timings_valid) {
        last_presence = nozzle_presence::unknown;
        ringdown.status = ringdown_status::aborted;
        ringdown.presence = nozzle_presence::unknown;
        ringdown.peak_mv = 0;
        if (report) {
            sendf("indx_nozzle_presence clock=%u status=%c presence=%c "
                        "peak_mv=%u",
                        timer_read_time(), (uint32_t)ringdown_status::aborted,
                        (uint32_t)nozzle_presence::unknown, 0u);
        }
        return;
    }
    ringdown.start(report, dump);
}

bool
coil_driver::ringdown_active() {
    return ringdown.active();
}

void
coil_driver::ringdown_step(bool heating) {
    if (ringdown.active()) {
        ringdown.step();
        return;
    }
    if (!ringdown_cfg.enable || tuner.active() || !state.timings_valid)
        return;
    uint32_t period_ms = heating ? ringdown_cfg.heat_ms : ringdown_cfg.idle_ms;
    uint32_t due = ringdown.next_schedule_time + timer_from_us(period_ms * 1000);
    if (ringdown.next_schedule_time != 0 &&
            timer_is_before(timer_read_time(), due))
        return;
    start_ringdown(true, false);
}

nozzle_presence
coil_driver::get_nozzle_presence() {
    return last_presence;
}

void
coil_driver::report_ringdown_status() {
    sendf("indx_nozzle_presence clock=%u status=%c presence=%c peak_mv=%u",
                timer_read_time(), (uint32_t)ringdown.status,
                (uint32_t)ringdown.presence, ringdown.peak_mv);
}
