"""Objective ingredients from one episode's case_XXXX_flight.csv (~5 Hz).

CSV schema (written by aws_monte_carlo_11_edf.fly_ustol2):
  time_s, phase, alt_agl_m, airspeed, roll_deg, pitch_deg, climb_mps,
  throttle_pct, theta_cmd_deg, lat, lon, load_factor_nz, load_factor_total,
  motor3_pwm, motor7_pwm, yaw_rate_dps, roll_cmd_deg, rud_cmd_deg
Phases: GROUND CLIMB CRUISE YAW_DBL ROLL_DBL RECOVER APPROACH FLARE ROLLOUT.

Everything here is pure-numpy on that CSV plus the _run_case result dict, so
it runs inside the worker process right after the episode and the heavy FDM
log can be deleted immediately after.
"""
import csv
import math

import numpy as np

# DT-split channel PWM rails (SERVOx min/max are 1000/2000 on this setup);
# within RAIL_MARGIN of a rail counts as saturated.
_PWM_LO, _PWM_HI, _RAIL_MARGIN = 1000, 2000, 25

_DOUBLET_PHASES = ("YAW_DBL", "ROLL_DBL")
_FLIGHT_PHASES  = ("CLIMB", "CRUISE") + _DOUBLET_PHASES + ("RECOVER", "APPROACH", "FLARE")


def _read(csv_path):
    cols = {}
    try:
        with open(csv_path, newline="") as f:
            rows = list(csv.DictReader(f))
    except OSError:
        return None
    if not rows:
        return None
    keys = rows[0].keys()
    for k in keys:
        if k == "phase":
            cols[k] = np.array([r.get(k, "") for r in rows])
        else:
            vals = []
            for r in rows:
                try:
                    vals.append(float(r.get(k, "") or 0.0))
                except ValueError:
                    vals.append(0.0)
            cols[k] = np.array(vals)
    return cols


def _reversal_rate(t, y):
    """Sign reversals of the detrended signal's derivative per second — a
    cheap oscillation-buzz detector (mirrors DASHBOARD/failure_criteria.py)."""
    if len(y) < 5:
        return 0.0
    dy = np.diff(y)
    sign = np.sign(dy)
    sign = sign[sign != 0]
    if len(sign) < 2:
        return 0.0
    reversals = int(np.sum(sign[1:] != sign[:-1]))
    span = max(t[-1] - t[0], 1e-6)
    return reversals / span


def _band_fraction(t, y, cutoff_hz=0.8):
    """Fraction of (detrended) signal power above cutoff_hz — D-gain buzz that
    summary statistics miss. Uses a mean-dt FFT (telemetry is ~uniform 5 Hz)."""
    n = len(y)
    if n < 16:
        return 0.0
    dt = max(np.mean(np.diff(t)), 1e-3)
    yd = y - np.polyval(np.polyfit(t, y, 1), t)
    spec = np.abs(np.fft.rfft(yd * np.hanning(n))) ** 2
    freqs = np.fft.rfftfreq(n, dt)
    total = float(np.sum(spec[1:]))
    if total <= 0:
        return 0.0
    return float(np.sum(spec[freqs > cutoff_hz]) / total)


def _cross_track_m(lat, lon, home_lat, home_lon, runway_hdg_deg):
    """Signed lateral offset (m) from the runway centreline through home."""
    dn = (lat - home_lat) * 111_320.0
    de = (lon - home_lon) * 111_320.0 * math.cos(math.radians(home_lat))
    h = math.radians(runway_hdg_deg)
    return de * math.cos(h) - dn * math.sin(h)


def _seg_stats(mask, err):
    if not np.any(mask):
        return None, None
    e = np.abs(err[mask])
    return float(np.max(e)), float(np.mean(e))


def compute(csv_path: str, result: dict, home_lat: float, home_lon: float,
            runway_hdg: float, cruise_alt: float = 100.0) -> dict:
    """-> flat metrics dict (None where a phase never ran). Never raises."""
    m = {}
    try:
        c = _read(csv_path)
        if c is None:
            return {"csv_missing": True}
        t     = c["time_s"]
        phase = c["phase"]
        air   = phase != "GROUND"

        # ---- tracking: doublet response --------------------------------
        roll_m = phase == "ROLL_DBL"
        yaw_m  = phase == "YAW_DBL"
        m["roll_err_max_deg"], m["roll_err_mean_deg"] = _seg_stats(
            roll_m, c["roll_deg"] - c["roll_cmd_deg"])
        if np.any(yaw_m):
            m["yaw_peak_rate_dps"] = float(np.max(np.abs(c["yaw_rate_dps"][yaw_m])))
            m["yaw_bank_max_deg"]  = float(np.max(np.abs(c["roll_deg"][yaw_m])))
        m["n_recover_events"] = int(np.sum((phase[1:] == "RECOVER")
                                           & (phase[:-1] != "RECOVER")))
        m["doublets_flown"] = bool(np.any(roll_m) and np.any(yaw_m))
        dbl_m = np.isin(phase, _DOUBLET_PHASES)
        if np.any(dbl_m):
            m["doublet_alt_sag_m"] = float(max(0.0, cruise_alt - np.min(c["alt_agl_m"][dbl_m])))

        # ---- engine-out recovery (climb+cruise: failure active from boot) --
        cc = np.isin(phase, ("CLIMB", "CRUISE")) & (c["alt_agl_m"] > 2.0)
        if np.any(cc):
            m["climb_yaw_rate_max_dps"] = float(np.max(np.abs(c["yaw_rate_dps"][cc])))
            m["climb_roll_max_deg"]     = float(np.max(np.abs(c["roll_deg"][cc])))
            has_fix = cc & (np.abs(c["lat"]) > 1e-6)
            if np.any(has_fix):
                xt = np.array([_cross_track_m(la, lo, home_lat, home_lon, runway_hdg)
                               for la, lo in zip(c["lat"][has_fix], c["lon"][has_fix])])
                m["track_dev_max_m"] = float(np.max(np.abs(xt)))

        # ---- control saturation ----------------------------------------
        flight = np.isin(phase, _FLIGHT_PHASES)
        if np.any(flight):
            m3, m7 = c["motor3_pwm"][flight], c["motor7_pwm"][flight]
            active = (m3 > 0) & (m7 > 0)                    # 0 until SERVO stream starts
            if np.any(active):
                sat = ((m3[active] >= _PWM_HI - _RAIL_MARGIN) |
                       (m3[active] <= _PWM_LO + _RAIL_MARGIN) |
                       (m7[active] >= _PWM_HI - _RAIL_MARGIN) |
                       (m7[active] <= _PWM_LO + _RAIL_MARGIN))
                m["dt_sat_frac"] = float(np.mean(sat))
            m["thr_sat_frac"] = float(np.mean(c["throttle_pct"][flight] >= 95.0))

        # ---- oscillation health (roll + yaw-rate channels, airborne) ----
        if int(np.sum(air)) > 16:
            ta = t[air]
            m["roll_reversal_hz"]  = _reversal_rate(ta, c["roll_deg"][air])
            m["yaw_reversal_hz"]   = _reversal_rate(ta, c["yaw_rate_dps"][air])
            m["roll_band_frac"]    = _band_fraction(ta, c["roll_deg"][air])
            m["yaw_band_frac"]     = _band_fraction(ta, c["yaw_rate_dps"][air])

        # ---- landing: touchdown state = last FLARE sample ----------------
        fl = np.where(phase == "FLARE")[0]
        if len(fl):
            i = fl[-1]
            m["td_sink_mps"]  = float(max(0.0, -c["climb_mps"][i]))
            m["td_pitch_deg"] = float(c["pitch_deg"][i])
            m["td_speed_mps"] = float(c["airspeed"][i])
            m["flare_max_sink_mps"] = float(max(0.0, -np.min(c["climb_mps"][fl])))
        ap = phase == "APPROACH"
        if np.any(ap):
            m["approach_max_sink_mps"] = float(max(0.0, -np.min(c["climb_mps"][ap])))
    except Exception as exc:                                # metrics must never kill an episode
        m["metrics_error"] = repr(exc)

    # ---- pass-throughs from the _run_case result dict --------------------
    for k in ("ground_roll_m", "landing_err_m", "landing_roll_m",
              "max_load_factor_total", "min_load_factor_nz", "max_roll_deg"):
        m[k] = result.get(k)
    return m
