#include "AP_DiffThrust.h"

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

/*
  uSTOL spanwise moment arms [m] for the nine thrust channels, pair-averaged
  over each channel's same-side EDF pair, ordered port -> centre -> starboard
  and index-aligned with k_motor1..k_motor9. Channel index 4 (k_motor5) is the
  centre mirror at y=0 and never differentiates. Starboard is +y.

  These are the per-channel averages of the SITL FDM EDF layout (s_motor[i].
  rotor_xyz[1] in LAT_SIM_Runner.cpp: 9 EDFs/side at 0.125 m pitch, 0.585..1.585 m
  from centreline), so the yaw moment the mixer commands is realised by the FDM
  at the SAME arms (no allocation-vs-plant gain error). If the FDM EDF geometry
  changes, update these to match. Mirror-consistent: ch1<->ch9, ch2<->ch8,
  ch3<->ch7, ch4<->ch6.
*/
const float AP_DiffThrust::_y_ch[AP_DiffThrust::NUM_CH] = {
    -1.585f, -1.3975f, -1.1475f, -0.8975f, 0.0f,
    +0.8975f, +1.1475f, +1.3975f, +1.585f
};

/*
  Channels that carry the differential-thrust yaw split. Only ch3 (index 2,
  EDF4,5, y=-1.1475) and its mirror ch7 (index 6, EDF14,15, y=+1.1475) are
  used - a symmetric mid-span pair, so the split stays thrust-neutral. Every
  other channel receives only its base/engine-out throttle. Set all entries
  true to restore the original all-channel split.
*/
const bool AP_DiffThrust::_dt_split_ch[AP_DiffThrust::NUM_CH] = {
    false, false, true, false, false, false, true, false, false
};

const AP_Param::GroupInfo AP_DiffThrust::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Differential thrust enable
    // @Description: Enable the velocity-scheduled differential-thrust yaw mixer for the 9-channel uSTOL airframe. When 0 (default) the mixer is a no-op and the firmware behaves as stock. When 1 the module drives SERVO functions k_motor1..k_motor9.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_DiffThrust, _enable, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: DT_VLO
    // @DisplayName: Diff-thrust full-authority airspeed
    // @Description: At or below this airspeed the velocity weight is 1 (full differential-thrust yaw authority).
    // @Units: m/s
    // @Range: 0 30
    // @User: Advanced
    AP_GROUPINFO("DT_VLO", 2, AP_DiffThrust, _dt_vlo, 13.0f),

    // @Param: DT_VHI
    // @DisplayName: Diff-thrust zero-authority airspeed
    // @Description: At or above this airspeed the velocity weight is 0 (uniform throttle, stock yaw handling). Should be greater than UST_DT_VLO.
    // @Units: m/s
    // @Range: 0 40
    // @User: Advanced
    AP_GROUPINFO("DT_VHI", 3, AP_DiffThrust, _dt_vhi, 17.0f),

    // @Param: NDES_MAX
    // @DisplayName: Diff-Thrust peak yaw moment
    // @Description: Maximum yaw moment (N*m) commanded to the diff thrust at full normalised yaw demand (yaw_n = 1) and full velocity weight, replaces the dimensionless DT_KYAW gain
    // @units: N*m
    // @Range: 0 50
    // @User: Advanced
    AP_GROUPINFO("NDES_MAX", 4, AP_DiffThrust, _dt_ndes_max, 20.0f),

    // @Param: DT_RLFF
    // @DisplayName: Diff-thrust roll feedforward
    // @Description: Aileron feedforward gain to pre-empt the parasitic blown-lift roll induced by yaw-only differential thrust. 0 disables (default).
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("DT_RLFF", 5, AP_DiffThrust, _dt_rlff, 0.0f),

    // @Param: UMAX
    // @DisplayName: Diff-thrust per-motor command ceiling
    // @Description: Hard upper bound on the per-motor normalised throttle command (0..1), e.g. a wire/current cap. Clamped internally to 0.05..1.0.
    // @Range: 0.05 1.0
    // @User: Advanced
    AP_GROUPINFO("UMAX", 6, AP_DiffThrust, _umax, 0.9f),

    // @Param: KRUD
    // @DisplayName: Rudder yaw-authority coefficient
    // @Description: Coefficient of the V^2-scaled aerodynamic rudder yaw authority, N_rud_avail = UST_KRUD * V^2 [N*m]. When >0 the mixer runs a true rudder/diff-thrust daisy-chain: the rudder is assumed to cover the yaw demand up to N_rud_avail and differential thrust supplies only the residual, so DT fades out on its own as airspeed (hence rudder authority) rises. When 0 (default) the legacy UST_DT_VLO/VHI velocity-weight schedule is used instead. Typical value ~0.11 (from ~25 N*m rudder authority at 15 m/s).
    // @Units: N*m/(m/s)/(m/s)
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("KRUD", 7, AP_DiffThrust, _k_rud, 0.0f),

    // @Param: UMIN
    // @DisplayName: Diff-thrust per-motor command floor
    // @Description: Lower bound on the per-motor normalised throttle command (0..1) for ALIVE channels, e.g. to keep EDFs above a spin-up idle. A failed/dead channel is still commanded to 0 regardless. Clamped internally to 0..UST_UMAX.
    // @Range: 0.0 0.5
    // @User: Advanced
    AP_GROUPINFO("UMIN", 8, AP_DiffThrust, _umin, 0.0f),

    // @Param: ASND
    // @DisplayName: Speed of sound for thrust map
    // @Description: Speed of sound [m/s] used for the blade tip Mach term in the throttle->thrust map. Match to the SITL FDM (vehcle.sound_speed).
    // @Units: m/s
    // @Range: 300 360
    // @User: Advanced
    AP_GROUPINFO("ASND", 9, AP_DiffThrust, _a_sound, 340.0f),

    AP_GROUPEND
};

AP_DiffThrust::AP_DiffThrust()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_DiffThrust::set_dt_active(bool active)
{
    if (active == _dt_active) {
        return;
    }
    _dt_active = active;
    gcs().send_text(MAV_SEVERITY_INFO, "uSTOL DT: %s", active ? "ACTIVE" : "PASSTHROUGH");
}

void AP_DiffThrust::ensure_ranges()
{
    // k_motor1..k_motor9 have no case in SRV_Channel_aux.cpp aux_servo_function_setup(),
    // so their output range is never established and a bare set_output_scaled() would
    // peg every ESC's high_out at its uninitialized default - saturating to SERVOn_MAX
    // for any nonzero scaled command. set_range() only touches channels whose function
    // is CURRENTLY k_motorN, so calling this once on the first update() (as before) races
    // the SERVOx_FUNCTION param assignment: when UST_ENABLE defaults to 1 at compile time,
    // update() starts running before a ground-station/script has a chance to reassign
    // SERVO1..9_FUNCTION away from their boot default (k_throttle), so the one-shot call
    // finds no matching channels, sets nothing, and never retries. Calling it every loop
    // is a fixed 9-iteration int-field write - negligible cost - and guarantees the range
    // is correct regardless of when SERVOx_FUNCTION gets (re)assigned.
    for (uint8_t k = 0; k < NUM_CH; k++) {
        SRV_Channels::set_range(SRV_Channels::get_motor_function(k), 1000);
    }
}

void AP_DiffThrust::update(bool airspeed_valid, float airspeed, AP_DT_EngineOut &fail)
{
    if (_enable == 0) {
        return;                                 // disabled: no-op, firmware behaves as stock
    }
    ensure_ranges();

    // base uniform throttle demand, 0..1 (k_throttle scaled output is 0..100). This is
    // populated by the throttle controller every loop regardless of whether a physical
    // channel is assigned to k_throttle, so it is a robust base on an airframe whose
    // ESCs live on k_motor1..k_motor9.
    const float base = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * 0.01f, 0.0f, 1.0f);
    if (!_dt_active) {
        // pilot's DT switch is off: plain equal throttle on all nine channels,
        // identical to an aircraft with no differential-thrust system at all.
        // USTF_MASK is deliberately ignored here - this is the "revert to dumb
        // throttle" fallback.
        for (uint8_t k = 0; k < NUM_CH; k++) {
            SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(k), base * 1000.0f);
        }
        return;
    }

    fail.check_and_update();

    // velocity weight w(V): 1 at/below VLO, 0 at/above VHI. Invalid airspeed -> VHI -> w=0.
    float vlo = _dt_vlo;
    float vhi = _dt_vhi;
    if (vhi < vlo + 0.1f) {
        vhi = vlo + 0.1f;                       // param sanity: no div-by-zero / inverted schedule
    }
    const float V = airspeed_valid ? airspeed : vhi;
    float w = constrain_float((vhi - V) / (vhi - vlo), 0.0f, 1.0f);

    // ground/idle guard: at V~0 (parked/taxiing) w would be 1; suppress the split below a
    // small throttle floor so the aircraft gets no differential yaw on the ground.
    if (base < 0.05f) {
        w = 0.0f;
    }

    // stock rudder demand normalised to [-1, 1] (+ = nose-right).
    const float yaw_n = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) / 4500.0f, -1.0f, 1.0f);
    const float u_cap = constrain_float(_umax, 0.05f, 0.95f);
    const float u_floor = constrain_float(_umin, 0.0f, u_cap);   // per-motor command floor (alive channels only)

    // antisymmetric per-channel split: nose-right (+yaw_n) cuts starboard (+y) and adds
    // port (-y), giving a nose-right yaw moment. Magnitude scales with the span arm; the
    // centre channel (y=0) never differentiates. All nine channels are always written.
    
    // for (uint8_t k = 0; k < NUM_CH; k++) {
    //     const SRV_Channel::Function fn = SRV_Channels::get_motor_function(k);
    //     const float du = w * _dt_kyaw * yaw_n * (-_y_ch[k] / _y_max);
    //     const float u  = constrain_float(base + du, 0.0f, u_cap);
    //     SRV_Channels::set_output_scaled(fn, u * 1000.0f);
    // }


    // --- thrust neutral allocation ---
     // Quadratic throttle->thrust map:  T(d) = A2*d^2 + A1*d  at current airspeed.
    // Derived from: T = rho*n^2*D^4*CT,  CT = CT1 + CT_J*J + CT_JM*J*Mtip,

    const float D3 = _D * _D * _D;
    const float D4 = D3 * _D;
    // Use the guarded speed V (falls back to VHI on invalid airspeed) so the map can
    // never be fed a stale/garbage sensor value.
    const float A2 = _rho * _n_max_rps * _n_max_rps * D4 * (_CT1 + _CT_JM * 3.14159265f * V / _a_sound);
    const float A1 = _rho * _n_max_rps * D3 * V * (_CT_J);
    const float T0 = A2 * base * base + A1 * base;

    // Total commanded yaw moment at the current rudder demand [N*m].
    const float N_cmd = yaw_n * _dt_ndes_max;

    // Split N_cmd between the aerodynamic rudder and differential thrust.
    //  - UST_KRUD > 0 : true daisy-chain. The rudder is assumed to carry the demand up
    //    to its V^2-scaled authority N_rud_avail; diff thrust supplies only the residual.
    //    This makes DT fade out on its own as airspeed (rudder authority) rises, replacing
    //    the VLO/VHI velocity-weight proxy.
    //  - UST_KRUD == 0 : legacy behaviour - DT carries the whole demand scaled by w(V).
    float N_dt;
    if (_k_rud > 0.0f) {
        const float N_rud_avail = _k_rud * V * V;
        N_dt = N_cmd - constrain_float(N_cmd, -N_rud_avail, N_rud_avail);
        if (base < 0.05f) {
            N_dt = 0.0f;                        // ground/idle guard (mirrors the w=0 guard)
        }
    } else {
        N_dt = N_cmd * w;
    }
//----------------------------------------------NOT WORKING---------------------------------------------
    // Thrust-neutral spanwise allocation. Because Sum(y_j)=0 the split adds zero net thrust.
    // Sign: +yaw_n = nose-right -> k_alloc < 0 -> port (y<0) thrust UP, stbd DOWN.
    //const float k_alloc = -N_dt / _sum_y_sq; // dT_j = k_alloc * y_j
//--------------------------------------------------------------------------------------
    const float T_floor = 0.05f;                        // min deliverable thrust per edf 
    const float T_ucap = A2*u_cap*u_cap + A1*u_cap;     // max deliverable thrust per edf (at throttle cmd ceiling)
    const float dT_max = 0.8f * MAX(0.0f, MIN(T0 - T_floor, T_ucap - T0));

    // Sum(y^2) and max|y| over the DT-participating channels (ch3/ch7) that are
    // currently alive. Only these carry the yaw split, so the denominator and the
    // saturation clamp are computed over that restricted set. If both DT channels
    // are dead, sum_y_sq==0 -> k_alloc==0 (no diff-thrust yaw; rely on the rudder).
    float sum_y_sq = 0.0f;
    float y_max_dt = 0.0f;
    for (uint8_t k = 0; k < NUM_CH; k++) {
        if (_dt_split_ch[k] && fail.channel_alive(k)) {
            sum_y_sq += _y_ch[k] * _y_ch[k];
            y_max_dt = MAX(y_max_dt, fabsf(_y_ch[k]));
        }
    }
    float k_alloc = is_positive(sum_y_sq) ? (-N_dt / sum_y_sq) : 0.0f;
    const float k_alloc_max = is_positive(y_max_dt) ? (dT_max / y_max_dt) : 0.0f;
    k_alloc = constrain_float(k_alloc, -k_alloc_max, k_alloc_max);
    



    for (uint8_t k = 0; k < NUM_CH; k++){
        const SRV_Channel::Function fn = SRV_Channels::get_motor_function(k);

        if (!fail.channel_alive(k)) {
            SRV_Channels::set_output_scaled(fn, 0.0f);   // declared dead / commanded off
            continue;
        }

        // Target thrust for this channel (engine-out gain applied first), then invert
        // quadratic for throttle. The yaw split is applied only on the DT-participating
        // channels (ch3/ch7); all others carry base/engine-out throttle alone.
        const float yaw_term = _dt_split_ch[k] ? (k_alloc * _y_ch[k]) : 0.0f;
        const float Tj = fail.thrust_gain(k) * T0 + yaw_term;
        const float disc = A1*A1 + 4.0f*A2*Tj ;
        float u;
        if (disc >= 0.0f && A2 > 1e-6f) {
            const float root_hi = (-A1 + sqrtf(disc)) / (2.0f * A2);
            const float root_lo = (-A1 - sqrtf(disc)) / (2.0f * A2);
            // T(d) = A2*d^2 + A1*d is a parabola with a minimum at d* = -A1/(2*A2)
            // (~0.15-0.26 across this airframe's flight envelope), so it is NOT
            // monotonic on [0,1]: any thrust achievable by a low throttle d < d*
            // (idle/approach/flare) has a mirror-image second solution at d' > d*.
            // Always taking the "+" root silently snapped every low-throttle
            // command onto the wrong, high-throttle branch. Pick whichever root
            // sits closer to the commanded base so a small yaw/engine-out offset
            // from base stays on the same branch base is on.
            u = (fabsf(root_lo - base) <= fabsf(root_hi - base)) ? root_lo : root_hi;
        } else {
            u = 0.0f;                                  // Tj below map minimum -> idle
        }
        u = constrain_float(u, u_floor, u_cap);
        SRV_Channels::set_output_scaled(fn, u * 1000.0f);
    }


    // optional aileron roll feedforward to pre-empt the blown-lift parasitic roll (off by default).
    // Scaled by the *actual* applied DT yaw moment (N_dt/NDES_MAX), not by w*yaw_n, so it stays
    // correct under both the legacy w(V) schedule and the UST_KRUD daisy-chain. In legacy mode
    // N_dt = yaw_n*NDES_MAX*w, so N_dt/NDES_MAX == w*yaw_n and this reduces to the original form.
    // The induced roll is measured (MATLAB) at ~1-2x the yaw moment; calibrate UST_DT_RLFF to match.
    if (!is_zero(_dt_rlff.get()) && !is_zero(_dt_ndes_max.get())) {
        const float dail = -_dt_rlff.get() * (N_dt / _dt_ndes_max.get()) * 4500.0f;
        const float ail0 = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, constrain_float(ail0 + dail, -4500.0f, 4500.0f));
    }
}

bool AP_DiffThrust::arming_checks(size_t buflen, char *buffer) const
{
    if (_enable == 0) {
        return true;                            // disabled: nothing to enforce
    }
    // Every k_motor1..k_motor9 must be assigned to a physical output, else the
    // mixer's writes vanish and the ESCs keep whatever else drives them - a
    // silent no-op that looks like "differential thrust does nothing".
    for (uint8_t k = 0; k < NUM_CH; k++) {
        if (!SRV_Channels::function_assigned(SRV_Channels::get_motor_function(k))) {
            hal.util->snprintf(buffer, buflen, "UST_ENABLE=1 but k_motor%u unassigned", (unsigned)(k + 1));
            return false;
        }
    }
    // Velocity schedule must not be inverted (update() self-heals this, but flag
    // it so a mis-set tune is caught on the bench rather than silently nudged).
    if (_dt_vhi < _dt_vlo + 0.1f) {
        hal.util->snprintf(buffer, buflen, "UST_DT_VHI must exceed UST_DT_VLO");
        return false;
    }
    return true;
}
