#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

/*
  AP_DT_EngineOut - rule-based engine-out reallocation for the 18-EDF / 9-channel
  uSTOL airframe. Declares which of the 18 EDFs are dead (USTF_MASK) and turns
  that into a per-channel thrust-gain table that AP_DiffThrust::update() applies
  on top of its existing yaw/roll mixer. The table is only recomputed when the
  mask or tuning parameters change - all per-loop queries are O(1) array reads.

  Physical layout (port wingtip -> starboard wingtip), channel : EDF count:
    ch1:1  ch2:2  ch3:2  ch4:2  ch5:4(straddles fuselage)  ch6:2  ch7:2  ch8:2  ch9:1
  Mirrors: ch1<->ch9, ch2<->ch8, ch3<->ch7, ch4<->ch6, ch5 self (centreline).
  ch2/ch8 blow directly over the ailerons.

  Registered as its own ParametersG2 subgroup (prefix USTF_), independent of
  AP_DiffThrust's UST_ tree, so it carries no AP_Param nesting-depth risk.
  AP_DiffThrust reaches it via a reference passed into update() each loop.
*/
class AP_DT_EngineOut {
public:
    AP_DT_EngineOut();
    CLASS_NO_COPY(AP_DT_EngineOut);

    // cheap per-loop poll: recomputes the rule table only if USTF_MASK/ABST/MRED/OPTS
    // changed since the last call. O(1) when nothing changed.
    void check_and_update();

    bool  failure_active() const { return _active; }
    bool  channel_alive(uint8_t k) const { return _alive[k]; }
    float thrust_gain(uint8_t k) const { return _q[k]; }
    float sum_y_sq_alive() const { return _sum_y_sq_alive; }

    // future auto-detector (ESC-telem RPM) writes failures through here too.
    void set_failure_mask(uint32_t mask, const char *source);

    // refuses to arm if USTF_MASK != 0 unless USTF_OPTS bit0 is set. Passes when mask==0.
    bool arming_checks(size_t buflen, char *buffer) const;

    static const struct AP_Param::GroupInfo var_info[];

private:
    static const uint8_t NUM_CH = 9;

    // ---- parameters ----
    AP_Int32 _mask;   // USTF_MASK  bit (e-1) = EDF e failed, e=1..18, port tip -> stbd tip
    AP_Float _abst;   // USTF_ABST  mirror thrust-boost fraction, aileron channel fully out
    AP_Float _mred;   // USTF_MRED  mirror thrust multiplier, mid channel fully out
    AP_Int16 _opts;   // USTF_OPTS  bit0 allow arm with mask!=0
                      //            bit1 apply survivor-boost to partial mid channels too
                      //            bit2 disable total-thrust make-up

    // ---- confirmed physical grouping (port tip -> stbd tip) ----
    static const uint8_t _n_ch[NUM_CH];     // EDF count per channel
    static const uint8_t _edf_lo[NUM_CH];   // first EDF number (1-based) of each channel
    static const float   _y_ch[NUM_CH];     // spanwise arm per channel [m] (same geometry as AP_DiffThrust)
    static uint8_t mirror(uint8_t k) { return 8 - k; }

    // ---- change detection ----
    int32_t _applied_mask = -1;
    float   _applied_abst = -1.0f;
    float   _applied_mred = -1.0f;
    int16_t _applied_opts = -1;

    // ---- derived (recomputed in recompute() only) ----
    bool  _active;
    bool  _alive[NUM_CH];
    float _q[NUM_CH];
    float _sum_y_sq_alive;

    void recompute();
    void announce() const;
};
