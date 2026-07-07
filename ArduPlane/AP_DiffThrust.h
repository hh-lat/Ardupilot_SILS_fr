#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include "AP_DT_EngineOut.h"

/*
  AP_DiffThrust - velocity-scheduled differential-thrust yaw mixer for the
  9-channel uSTOL distributed-propulsion airframe.

  In normal flight the stock rudder yaw demand is converted into an
  antisymmetric per-motor thrust split that fades in as airspeed drops
  (an aerodynamic rudder loses authority at low speed). The split is
  allocated in THRUST space (thrust-neutral: sum of the per-channel deltas
  is zero) and inverted through the prop's quadratic throttle->thrust map,
  so a yaw command produces no net axial-thrust (speed) perturbation.

  Two scheduling modes select how much of the yaw demand DT carries:
    - UST_KRUD > 0 : true rudder/DT daisy-chain. The aerodynamic rudder is
      assumed to cover the demand up to its V^2-scaled authority
      (N_rud_avail = UST_KRUD*V^2) and DT supplies only the residual, so DT
      fades out on its own as airspeed rises.
    - UST_KRUD == 0 : legacy velocity-weight proxy. DT carries the whole
      demand scaled by w(V): full at/below UST_DT_VLO, zero at/above
      UST_DT_VHI (uniform stock throttle).

  Scope: a clean, self-contained re-implementation (ArduPlane 4.5.x/4.7.x) of
  the validated "DT-2" yaw mixer ONLY - no SITL/FDM, QP allocator, or engine-out
  logic. It lives in the vehicle directory (not libraries/) so no build-system
  change is needed, and it is purely additive: when UST_ENABLE=0 it is a no-op
  and the firmware behaves exactly as stock.

  When UST_ENABLE=1 the module becomes the sole authority for SERVO functions
  k_motor1..k_motor9 and writes all nine every loop (uniform stock throttle at
  high speed / zero rudder; antisymmetric split at low speed). The aircraft's
  nine ESCs must therefore be assigned SERVOx_FUNCTION = k_motor1..k_motor9
  before enabling. Hooked from Plane::servos_output().
*/
class AP_DiffThrust {
public:
    AP_DiffThrust();

    CLASS_NO_COPY(AP_DiffThrust);

    // per-loop mixer. airspeed is passed in (rather than including AP_AHRS) to
    // keep the module decoupled. It returns immediately when disabled. When
    // airspeed_valid is false the schedule falls back to the safe uniform state.
    // fail supplies the current engine-out rule table (see AP_DT_EngineOut); pass
    // plane.g2.engine_out from the call site.
    void update(bool airspeed_valid, float airspeed, AP_DT_EngineOut &fail);

    bool enabled() const { return _enable != 0; }

    // master RC-switch toggle: false forces plain equal-throttle passthrough on
    // all nine channels (the "no-DT aircraft" fallback), regardless of USTF_MASK.
    void set_dt_active(bool active);
    bool dt_active() const { return _dt_active; }

    // pre-arm sanity check (mirrors the quadplane motors->arming_checks pattern).
    // When enabled, refuses to arm if any of the nine k_motor functions is not
    // assigned to a physical output (the mixer would otherwise be a silent no-op
    // - the ESCs would stay on whatever still drives them) or if the velocity
    // schedule is inverted. Returns true (pass) when disabled. Fills buffer with a
    // human-readable reason on failure.
    bool arming_checks(size_t buflen, char *buffer) const;

    static const struct AP_Param::GroupInfo var_info[];

private:
    // true unless the pilot has flipped the DT RC switch off; see set_dt_active().
    bool _dt_active = true;

    // ---- parameters (UST_ prefix applied by the ParametersG2 subgroup) ----
    AP_Int8  _enable;       // UST_ENABLE
    AP_Float _dt_vlo;       // UST_DT_VLO   [m/s]
    AP_Float _dt_vhi;       // UST_DT_VHI   [m/s]
    AP_Float _dt_ndes_max ;      // UST_DT_NDES_MAX
    AP_Float _dt_rlff;      // UST_DT_RLFF
    AP_Float _umax;         // UST_UMAX
    AP_Float _k_rud;        // UST_KRUD  rudder yaw-authority coeff [N*m/(m/s)^2]; 0 = legacy w(V) schedule
    AP_Float _umin;         // UST_UMIN  per-motor command floor (alive channels)
    AP_Float _a_sound;      // UST_ASND  speed of sound [m/s] (thrust-map blade tip Mach)

    // ---- uSTOL spanwise geometry (hardcoded; matches SITL FDM rotor_xyz; see .cpp) ----
    static const uint8_t NUM_CH = 9;
    static const float _y_ch[NUM_CH];           // pair-averaged span arm per channel [m]
    static constexpr float _y_max = 1.585f;     // max |_y_ch|

    // Channels that carry the differential-thrust yaw split. Restricted to ch3
    // (EDF4,5) and its mirror ch7 (EDF14,15); every other channel gets only its
    // base/engine-out throttle. (Original design used all channels - revert this
    // array to all-true to restore that.)
    static const bool _dt_split_ch[NUM_CH];
   
    // denominator for thrust-neutral allocation (sum of _y_ch^2 over all channels,
    // healthy). Legacy: the active split now sums over the alive DT channels at runtime.
    static constexpr float _sum_y_sq = 13.175f;
   
    // EDF prop constants for thrust-neutral allocation (hardcoded; match SITL FDM prop model)
    static constexpr float _rho = 1.15f;
    static constexpr float _n_max_rps = 200.0f;
    static constexpr float _D = 0.120f;
    static constexpr float _CT1 = 0.6917f;
    static constexpr float _CT_J = -0.7345f;
    static constexpr float _CT_JM = 1.6471f;

    // per-loop output-range setup for the nine motor channels (see .cpp)
    void ensure_ranges();
};
