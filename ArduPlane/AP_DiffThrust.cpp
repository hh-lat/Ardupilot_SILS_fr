#include "AP_DiffThrust.h"

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

/*
  uSTOL V1.3 Config-A spanwise moment arms [m] for the nine thrust channels,
  pair-averaged over each channel's same-side motor pair, ordered
  port -> centre -> starboard and index-aligned with k_motor1..k_motor9.
  Channel index 4 (k_motor5) is the centre mirror at y=0 and never
  differentiates. Starboard is +y. These reproduce the validated source
  geometry (MOTOR_Y / CH2MOT) exactly.
*/
const float AP_DiffThrust::_y_ch[AP_DiffThrust::NUM_CH] = {
    -1.5225f, -1.2725f, -1.0225f, -0.7725f, 0.0f,
    +0.7725f, +1.0225f, +1.2725f, +1.5225f
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

    AP_GROUPEND
};

AP_DiffThrust::AP_DiffThrust()
{
    AP_Param::setup_object_defaults(this, var_info);
    _range_inited = false;
}

void AP_DiffThrust::ensure_ranges()
{
    if (_range_inited) {
        return;
    }
    // k_motor1..k_motor9 have no case in SRV_Channel_aux.cpp aux_servo_function_setup(),
    // so their output range is never established and a bare set_output_scaled() would
    // peg every ESC to SERVOn_MIN. Establish a 0..1000 range so scaled commands map to
    // each channel's configured SERVOn_MIN..MAX (honouring per-channel reversal). Aux
    // functions are bound long before servos_output() first runs, so doing this lazily
    // on the first enabled update() is safe.
    for (uint8_t k = 0; k < NUM_CH; k++) {
        SRV_Channels::set_range(SRV_Channels::get_motor_function(k), 1000);
    }
    _range_inited = true;
}

void AP_DiffThrust::update(bool airspeed_valid, float airspeed)
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
    const float u_cap = constrain_float(_umax, 0.05f, 1.0f);

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
    const float A2 = _rho * _n_max_rps * _n_max_rps * D4 * (_CT1 + _CT_JM * 3.14159265f * airspeed/ _a_sound);
    const float A1 = _rho * _n_max_rps * D3 * airspeed * (_CT_J);
    const float T0 = A2 * base * base + A1 * base;
    
    // Commanded yaw moment scaled by velocity weight
     // Sign: +yaw_n = nose-right -> k_alloc < 0 -> port (y<0) thrust UP, stbd DOWN.
    const float N_des = yaw_n * _dt_ndes_max * w;
    const float k_alloc = -N_des / _sum_y_sq; // dT_j = k_alloc * y_j

    for (uint8_t k = 0; k < NUM_CH; k++){
        const SRV_Channel::Function fn = SRV_Channels::get_motor_function(k);
        // Target thrus t for this channel, then invert quadratic for throttle

        const float Tj = T0 + k_alloc * _y_ch[k];
        const float disc = A1*A1 + 4.0f*A2*Tj ;
        float u;
        if (disc >= 0.0f && A2 > 1e-6f) {
            u = (-A1 + sqrtf(disc)) / (2.0f * A2);
        } else {
            u = 0.0f;                                  // Tj below map minimum -> idle
        }
        u = constrain_float(u, 0.0f, u_cap);
        SRV_Channels::set_output_scaled(fn, u * 1000.0f);
    }


    // optional aileron roll feedforward to pre-empt the blown-lift parasitic roll (off by default).
    if (!is_zero(_dt_rlff.get())) {
        const float dail = -_dt_rlff.get() * w * yaw_n * 4500.0f;
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
