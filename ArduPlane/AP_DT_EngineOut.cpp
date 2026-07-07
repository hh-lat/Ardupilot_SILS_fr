#include "AP_DT_EngineOut.h"

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

const uint8_t AP_DT_EngineOut::_n_ch[AP_DT_EngineOut::NUM_CH]   = {1, 2, 2, 2, 4, 2, 2, 2, 1};
const uint8_t AP_DT_EngineOut::_edf_lo[AP_DT_EngineOut::NUM_CH] = {1, 2, 4, 6, 8, 12, 14, 16, 18};
// Per-channel spanwise arms [m], pair-averaged, port -> centre -> starboard.
// SAME geometry as AP_DiffThrust::_y_ch: the per-channel averages of the SITL FDM
// EDF layout (s_motor[i].rotor_xyz[1]). Keep the two arrays identical.
const float AP_DT_EngineOut::_y_ch[AP_DT_EngineOut::NUM_CH] = {
    -1.585f, -1.3975f, -1.1475f, -0.8975f, 0.0f,
    +0.8975f, +1.1475f, +1.3975f, +1.585f
};

const AP_Param::GroupInfo AP_DT_EngineOut::var_info[] = {

    // @Param: MASK
    // @DisplayName: Failed-EDF mask
    // @Description: Bitmask of failed EDFs, bit (n-1) = EDF n, numbered port wingtip (1) to starboard wingtip (18). Ground-settable; a future automatic detector would set the same bits.
    // @Bitmask: 0:EDF1,1:EDF2,2:EDF3,3:EDF4,4:EDF5,5:EDF6,6:EDF7,7:EDF8,8:EDF9,9:EDF10,10:EDF11,11:EDF12,12:EDF13,13:EDF14,14:EDF15,15:EDF16,16:EDF17,17:EDF18
    // @User: Advanced
    AP_GROUPINFO("MASK", 1, AP_DT_EngineOut, _mask, 0),

    // @Param: ABST
    // @DisplayName: Aileron-channel mirror boost fraction
    // @Description: When channel 2 or 8 (the aileron-blowing channels) is fully dead, the mirror channel's per-EDF thrust is multiplied by (1+USTF_ABST) to protect at least one working aileron.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("ABST", 2, AP_DT_EngineOut, _abst, 0.5f),

    // @Param: MRED
    // @DisplayName: Mid-channel mirror thrust multiplier
    // @Description: When a mid channel (3, 4, 6 or 7) is fully dead, the mirror channel's per-EDF thrust is multiplied by this factor to roughly match blown lift and trim roll.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("MRED", 3, AP_DT_EngineOut, _mred, 0.5f),

    // @Param: OPTS
    // @DisplayName: Engine-out rule options
    // @Description: Bit 0: allow arming with USTF_MASK != 0. Bit 1: also apply the survivor-boost treatment to partially-failed mid channels (3,4,6,7). Bit 2: disable the symmetric total-thrust make-up.
    // @Bitmask: 0:Allow arm with failure,1:Boost partial mid channels,2:Disable thrust make-up
    // @User: Advanced
    AP_GROUPINFO("OPTS", 4, AP_DT_EngineOut, _opts, 0),

    AP_GROUPEND
};

AP_DT_EngineOut::AP_DT_EngineOut()
{
    AP_Param::setup_object_defaults(this, var_info);
    for (uint8_t k = 0; k < NUM_CH; k++) {
        _alive[k] = true;
        _q[k] = 1.0f;
    }
    _active = false;
    _sum_y_sq_alive = 13.175f;   // matches AP_DiffThrust::_sum_y_sq when healthy (sum of _y_ch^2)
}

void AP_DT_EngineOut::check_and_update()
{
    if (_mask == _applied_mask && is_equal(_abst.get(), _applied_abst) &&
        is_equal(_mred.get(), _applied_mred) && _opts == _applied_opts) {
        return;   // nothing changed: tables already current
    }
    recompute();
    _applied_mask = _mask;
    _applied_abst = _abst.get();
    _applied_mred = _mred.get();
    _applied_opts = _opts;
    announce();
}

void AP_DT_EngineOut::set_failure_mask(uint32_t mask, const char *source)
{
    _mask.set(int32_t(mask));
    gcs().send_text(MAV_SEVERITY_INFO, "uSTOL EO: mask set by %s", source);
}

void AP_DT_EngineOut::recompute()
{
    const uint32_t mask = uint32_t(_mask.get());
    _active = (mask != 0);

    float h[NUM_CH];
    for (uint8_t k = 0; k < NUM_CH; k++) {
        uint8_t failed = 0;
        for (uint8_t e = 0; e < _n_ch[k]; e++) {
            const uint8_t edf = _edf_lo[k] + e;
            if (mask & (1UL << (edf - 1))) {
                failed++;
            }
        }
        const uint8_t working = _n_ch[k] - failed;
        h[k] = float(working) / float(_n_ch[k]);
        _alive[k] = (working > 0);
        _q[k] = 1.0f;
    }

    const bool boost_partial_mid = (_opts & 0x02) != 0;
    const bool disable_makeup    = (_opts & 0x04) != 0;

    // R1 - wingtip channels: total failure on ch1(idx0) or ch9(idx8) kills the mirror tip.
    static const uint8_t tip_idx[2] = {0, 8};
    for (uint8_t t = 0; t < 2; t++) {
        const uint8_t k = tip_idx[t];
        if (h[k] <= 0.0f) {
            const uint8_t m = mirror(k);
            if (_alive[m]) {
                _alive[m] = false;
                _q[m] = 0.0f;
            }
        }
    }

    // R2 - aileron channels ch2(idx1) / ch8(idx7).
    static const uint8_t ail_idx[2] = {1, 7};
    for (uint8_t t = 0; t < 2; t++) {
        const uint8_t k = ail_idx[t];
        const uint8_t m = mirror(k);
        if (h[k] > 0.0f && h[k] < 1.0f) {
            _q[k] *= 2.0f;                             // partial: survivor doubles thrust
        } else if (h[k] <= 0.0f && _alive[m]) {
            _q[m] *= (1.0f + _abst.get());              // total: boost the mirror aileron channel
        }
    }

    // R3 - mid channels ch3(idx2)/ch4(idx3)/ch6(idx5)/ch7(idx6).
    static const uint8_t mid_idx[4] = {2, 3, 5, 6};
    for (uint8_t t = 0; t < 4; t++) {
        const uint8_t k = mid_idx[t];
        const uint8_t m = mirror(k);
        if (h[k] > 0.0f && h[k] < 1.0f) {
            if (boost_partial_mid) {
                _q[k] *= 2.0f;
            }
        } else if (h[k] <= 0.0f && _alive[m]) {
            _q[m] *= _mred.get();                       // total: trim the mirror channel
        }
    }

    // R4 - centre channel ch5(idx4): never used for steering; any deficit just
    // feeds the make-up pass below.

    if (!disable_makeup) {
        float delivered = 0.0f;
        for (uint8_t k = 0; k < NUM_CH; k++) {
            if (_alive[k]) {
                delivered += _n_ch[k] * h[k] * _q[k];
            }
        }
        if (delivered > 0.01f) {
            const float lambda = MIN(18.0f / delivered, 2.5f);
            for (uint8_t k = 0; k < NUM_CH; k++) {
                if (_alive[k]) {
                    _q[k] *= lambda;
                }
            }
        }
    }

    _sum_y_sq_alive = 0.0f;
    for (uint8_t k = 0; k < NUM_CH; k++) {
        if (_alive[k]) {
            _sum_y_sq_alive += _y_ch[k] * _y_ch[k];
        }
    }
}

void AP_DT_EngineOut::announce() const
{
    const uint32_t mask = uint32_t(_mask.get());
    if (mask == 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "uSTOL EO: cleared");
    } else {
        uint8_t n_failed = 0;
        for (uint8_t e = 0; e < 18; e++) {
            if (mask & (1UL << e)) {
                n_failed++;
            }
        }
        gcs().send_text(MAV_SEVERITY_WARNING, "uSTOL EO: mask=0x%05lX (%u EDF out)",
                         (unsigned long)mask, (unsigned)n_failed);
    }
    AP::logger().WriteStreaming("UDTG", "TimeUS,Msk,SY2,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9",
                                 "QIffffffffff",
                                 AP_HAL::micros64(), mask, _sum_y_sq_alive,
                                 _q[0], _q[1], _q[2], _q[3], _q[4], _q[5], _q[6], _q[7], _q[8]);
}

bool AP_DT_EngineOut::arming_checks(size_t buflen, char *buffer) const
{
    if (_mask == 0 || (_opts & 0x01) != 0) {
        return true;
    }
    hal.util->snprintf(buffer, buflen, "USTF_MASK!=0 (EDF failure declared)");
    return false;
}
