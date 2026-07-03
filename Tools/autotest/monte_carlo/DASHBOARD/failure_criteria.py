"""failure_criteria.py — configurable, phase-aware failure-criteria engine for the MC dashboard.

Loaded by build_dashboard_data.py (pure numpy/pandas/scipy — no Streamlit). Reads a human-edited
JSON config (failure_criteria_config.json) of thresholds + grace intervals and grades each case:

  * scalar      — one value/case (a metric already computed by per_case) vs min/max -> ok/fail.
  * timeseries  — a per-sample signal vs bounds, with a GRACE interval: a brief excursion that
                  recovers within grace_s is only 'flag'ged; a sustained one is 'fail'. grace_s=0
                  means any excursion fails. Optionally scoped to one flight phase.
  * oscillation — per-phase control-command health. Time-domain activity (RMS / reversal-rate /
                  slew-saturation) is computed for EVERY phase the case flew; a 0-Nyquist spectral
                  band-power fraction is added only on long phases (N >= min_samples). Emits per-
                  (channel,phase) rows for the dashboard's per-phase view.

Per-case verdict: any sustained CRITICAL breach -> FAIL; else any sustained DEGRADED breach ->
PARTIAL; else PASS. 'na' (signal/phase/metric unavailable, or phase never flown) is always neutral.

Sampling note (measured across 145 cases): sim_output is uniform ~9.94 Hz -> Nyquist ~4.97 Hz, so
spectra cannot see oscillation above ~5 Hz (it aliases). The separate *_flight.csv (~3.3 Hz) is used
only for phase labels, never for spectra.
"""
import glob
import json
import os

import numpy as np
import pandas as pd

try:
    from scipy.signal import detrend as _sp_detrend, welch as _sp_welch
    _HAVE_SCIPY = True
except Exception:                                    # pragma: no cover - scipy is a dep, but degrade
    _HAVE_SCIPY = False

G = 9.81
_TRAP = getattr(np, "trapezoid", None) or getattr(np, "trapz", None)   # numpy 2.x renamed trapz
DEFAULT_CONFIG_NAMES = ["failure_criteria_config.json"]
_RANK = {"na": -1, "ok": 0, "flag": 1, "fail": 2}


# --------------------------------------------------------------------------- #
#  Config loading
# --------------------------------------------------------------------------- #
def load_criteria_config(outdir, names=None):
    """Find & load a failure-criteria config JSON. Searches outdir, outdir/.., outdir/../...

    `names` is the ordered list of filenames to try (first found wins). It defaults to
    DEFAULT_CONFIG_NAMES (the landing-mission config); build_dashboard_data.py passes the
    doublet config first for a doublet campaign so the right thresholds are used per test type.

    Returns (config_dict, criteria_keys). Returns (None, []) if the file is absent, disabled, or
    unparseable (prints one warning) — so the build keeps working with no criteria."""
    names = names or DEFAULT_CONFIG_NAMES
    seen = set()
    for d in (outdir, os.path.join(outdir, ".."), os.path.join(outdir, "..", "..")):
        for name in names:
            path = os.path.abspath(os.path.join(d, name))
            if path in seen:
                continue
            seen.add(path)
            if not os.path.exists(path):
                continue
            try:
                with open(path) as f:
                    cfg = json.load(f)
            except Exception as e:
                print(f"failure_criteria: WARNING could not parse {path}: {e} — criteria disabled")
                return None, []
            if not cfg.get("enabled", True):
                print(f"failure_criteria: {os.path.basename(path)} has enabled=false — criteria disabled")
                return None, []
            crit = cfg.get("criteria", {}) or {}
            cfg["_config_dir"] = os.path.dirname(path)   # so envelope_file paths resolve next to the config
            print(f"failure_criteria: loaded {len(crit)} criteria from {path}")
            return cfg, list(crit.keys())
    print("failure_criteria: no failure_criteria_config.json found — criteria skipped")
    return None, []


# --------------------------------------------------------------------------- #
#  Signal registry — name -> (required columns, df -> series in DISPLAY units)
# --------------------------------------------------------------------------- #
def _nz(df):
    t = df["Time_s"].values.astype(float)
    a_up = -np.gradient(df["V_ned_gnd_2"].values.astype(float), t)   # +up accel; mirrors landing_metrics
    return 1.0 + a_up / G


SIGNAL_DEFS = {
    "pitch_deg":        (["theta"],                 lambda df: np.degrees(df["theta"].values.astype(float))),
    "roll_deg":         (["phi"],                   lambda df: np.degrees(df["phi"].values.astype(float))),
    "alpha_deg":        (["alpha_deg"],             lambda df: df["alpha_deg"].values.astype(float)),
    "gamma_deg":        (["gamma_deg"],             lambda df: df["gamma_deg"].values.astype(float)),
    "alt_agl_m":        (["alt_agl_m"],             lambda df: df["alt_agl_m"].values.astype(float)),
    "TAS_mps":          (["TAS_mps"],               lambda df: df["TAS_mps"].values.astype(float)),
    "climb_mps":        (["V_ned_gnd_2"],           lambda df: -df["V_ned_gnd_2"].values.astype(float)),
    "descent_mps":      (["V_ned_gnd_2"],           lambda df:  df["V_ned_gnd_2"].values.astype(float)),
    # air-relative flight-path angle (deg) = pitch - AoA; pair with TAS for the air-relative
    # flight envelope (matches the still-air trim maps even when there is wind).
    "gamma_air_deg":    (["theta", "V_b_tas_0", "V_b_tas_2"],
                         lambda df: np.degrees(df["theta"].values.astype(float))
                                    - np.degrees(np.arctan2(df["V_b_tas_2"].values.astype(float),
                                                            df["V_b_tas_0"].values.astype(float)))),
    "p_dps":            (["p"],                     lambda df: np.degrees(df["p"].values.astype(float))),
    "q_dps":            (["q"],                     lambda df: np.degrees(df["q"].values.astype(float))),
    "r_dps":            (["r"],                     lambda df: np.degrees(df["r"].values.astype(float))),
    "n_z":              (["Time_s", "V_ned_gnd_2"], _nz),
    # actual control-surface deflections (deg) — used by the flight-envelope limit criteria
    "delta_e_deg":      (["delta_e"],               lambda df: np.degrees(df["delta_e"].values.astype(float))),
    "delta_aL_deg":     (["delta_aL"],              lambda df: np.degrees(df["delta_aL"].values.astype(float))),
    "delta_r_deg":      (["delta_r"],               lambda df: np.degrees(df["delta_r"].values.astype(float))),
    "delta_e_cmd_deg":  (["delta_e_cmd"],           lambda df: np.degrees(df["delta_e_cmd"].values.astype(float))),
    "delta_aL_cmd_deg": (["delta_aL_cmd"],          lambda df: np.degrees(df["delta_aL_cmd"].values.astype(float))),
    "delta_aR_cmd_deg": (["delta_aR_cmd"],          lambda df: np.degrees(df["delta_aR_cmd"].values.astype(float))),
    "delta_r_cmd_deg":  (["delta_r_cmd"],           lambda df: np.degrees(df["delta_r_cmd"].values.astype(float))),
    "thrust_cmd":       (["mot0_thr_cmd"],          lambda df: df["mot0_thr_cmd"].values.astype(float)),
}


def get_signal(df, name):
    """Return (t, y) in display units, or (None, None) if the source columns are absent/unknown."""
    spec = SIGNAL_DEFS.get(name)
    if spec is None:
        return None, None
    cols, fn = spec
    if any(c not in df.columns for c in cols):
        return None, None
    try:
        return df["Time_s"].values.astype(float), np.asarray(fn(df), dtype=float)
    except Exception:
        return None, None


# --------------------------------------------------------------------------- #
#  Phase timeline
# --------------------------------------------------------------------------- #
def _alt_crossing(t, a, thr, rising):
    """Time of the first rising (or falling) crossing of threshold `thr` in altitude series a."""
    a = np.asarray(a, dtype=float)
    if len(a) < 2:
        return float("nan")
    idx = (np.flatnonzero((a[:-1] <= thr) & (a[1:] > thr)) if rising
           else np.flatnonzero((a[:-1] > thr) & (a[1:] <= thr)))
    return float(np.asarray(t, dtype=float)[idx[0]]) if len(idx) else float("nan")


def _flight_time_offset(df, fl):
    """sim_output Time_s and flight.csv time_s are on DIFFERENT clocks: sim_output logs ~20 s of
    pre-arm/ground settling that the flight log omits, so flight time_s lags Time_s by a (verified)
    CONSTANT offset. Estimate the offset to ADD to flight time as the MEDIAN of the time gap at
    several sharp shared altitude events — liftoff (1 m rising), peak altitude, touchdown (1 m
    falling). The median is robust to a flat cruise plateau (ambiguous peak) or a missing touchdown,
    and is computed per-case from that case's own data (not tuned to any campaign). Returns 0.0 when
    not reliably estimable (e.g. the aircraft never climbed)."""
    cols = getattr(fl, "columns", [])
    if "alt_agl_m" not in cols or "alt_agl_m" not in df.columns:
        return 0.0
    st = df["Time_s"].values.astype(float); sa = df["alt_agl_m"].values.astype(float)
    ft = fl["time_s"].values.astype(float); fa = fl["alt_agl_m"].values.astype(float)
    if len(st) < 5 or len(ft) < 5 or np.nanmax(sa) < 10.0 or np.nanmax(fa) < 10.0:
        return 0.0
    offs = []
    for thr, rising in ((1.0, True), (1.0, False)):                 # liftoff, then touchdown
        ts, tf = _alt_crossing(st, sa, thr, rising), _alt_crossing(ft, fa, thr, rising)
        if np.isfinite(ts) and np.isfinite(tf):
            offs.append(ts - tf)
    offs.append(float(st[np.nanargmax(sa)] - ft[np.nanargmax(fa)]))  # peak (plateau-noisy; median guards)
    offs = [o for o in offs if np.isfinite(o)]
    if not offs:
        return 0.0
    off = float(np.median(offs))
    span = float(st.max() - st.min())
    return off if abs(off) <= span else 0.0


def _ground_before_liftoff(df, phase):
    """Force every sample BEFORE the aircraft first leaves the ground (AGL > 0.5 m) to 'GROUND',
    regardless of the flight-controller label. The controller calls the rotation / late ground roll
    'CLIMB', which would otherwise feed pitch=0 and climb_rate=0 GROUND samples into the CLIMB-scoped
    criteria and make their 'worst value' read 0 (spurious fails). Only the takeoff-side prefix is
    touched, so post-landing ROLLOUT is unaffected."""
    if "alt_agl_m" not in df.columns or len(phase) == 0:
        return phase
    phase = np.array(phase, dtype=object)            # ensure writable (a pandas .values view is read-only)
    alt = df["alt_agl_m"].values.astype(float)
    air = np.flatnonzero(alt > 0.5)
    if len(air):
        phase[:int(air[0])] = "GROUND"
    return phase


def build_phase_timeline(df, fl):
    """Per-row phase label aligned to df, by backward merge_asof of flight.csv phase onto Time_s.

    The flight log's clock is first aligned to the sim_output clock (see _flight_time_offset) — the
    two differ by ~20 s of pre-arm time, and joining on raw time would land every phase ~20 s early.
    Returns (phase_array, source) where source is 'flight' or 'fallback'. Falls back to a coarse
    proxy from plane_moving_state/altitude when flight.csv is missing or unusable."""
    t = df["Time_s"].values.astype(float)
    if (fl is not None and getattr(fl, "columns", None) is not None
            and "phase" in fl.columns and "time_s" in fl.columns and len(fl) >= 2):
        try:
            off = _flight_time_offset(df, fl)
            left = pd.DataFrame({"t": t, "_i": np.arange(len(t))}).sort_values("t")
            right = fl[["time_s", "phase"]].rename(columns={"time_s": "t"}).dropna(subset=["t"]).copy()
            right["t"] = right["t"] + off                  # align flight clock -> sim_output clock
            right = right.sort_values("t")
            merged = pd.merge_asof(left, right, on="t", direction="backward").sort_values("_i")
            phase = merged["phase"].astype(object).where(merged["phase"].notna(), "GROUND").values
            phase = np.asarray(phase, dtype=object)
            return _ground_before_liftoff(df, phase), "flight"
        except Exception as e:
            # genuine bad/missing flight.csv -> fall back; but make code bugs visible, not silent
            print(f"failure_criteria: WARNING phase join failed ({type(e).__name__}: {e}) — using fallback phases")
    return _fallback_phase(df), "fallback"


def _fallback_phase(df):
    """Coarse phase proxy from sim_output alone (used only when flight.csv is absent)."""
    n = len(df)
    ph = np.full(n, "CLIMB", dtype=object)
    alt = df["alt_agl_m"].values.astype(float) if "alt_agl_m" in df.columns else np.zeros(n)
    if n == 0:
        return ph
    apogee = int(np.argmax(alt))
    idx = np.arange(n)
    on_ground = alt < 2.0
    ph[(idx <= apogee) & ~on_ground] = "CLIMB"
    ph[(idx > apogee) & ~on_ground & (alt > 8.0)] = "APPROACH"
    ph[(idx > apogee) & ~on_ground & (alt <= 8.0)] = "FLARE"
    flew = np.cumsum(alt > 5.0) > 0                  # has been airborne by now
    ph[on_ground & flew] = "ROLLOUT"
    ph[on_ground & ~flew] = "GROUND"
    return ph


# --------------------------------------------------------------------------- #
#  Helpers
# --------------------------------------------------------------------------- #
def _safe_float(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def _bounds(c):
    """(lo, hi) from a criterion: max_abs:X -> (-X, X); else min/max (either may be None)."""
    if c.get("max_abs") is not None:
        a = abs(float(c["max_abs"]))
        return -a, a
    lo = float(c["min"]) if c.get("min") is not None else None
    hi = float(c["max"]) if c.get("max") is not None else None
    return lo, hi


def _worst_value(y, lo, hi):
    """The single most-violating sample (largest excess above hi, or deepest below lo)."""
    y = y[np.isfinite(y)]
    if len(y) == 0:
        return float("nan")
    cands = []
    if hi is not None:
        ymax = float(np.max(y)); cands.append((ymax - hi, ymax))
    if lo is not None:
        ymin = float(np.min(y)); cands.append((lo - ymin, ymin))
    if not cands:
        return float(y[np.argmax(np.abs(y))])
    cands.sort(key=lambda z: z[0], reverse=True)
    return cands[0][1]


def _scalar_status(v, lo, hi):
    if v is None or not np.isfinite(v):
        return "na"
    if lo is not None and v < lo:
        return "fail"
    if hi is not None and v > hi:
        return "fail"
    return "ok"


def _slice_phase(t, y, phase_arr, phase):
    if phase is None or str(phase).lower() in ("", "all", "null", "none"):
        return t, y
    mask = (phase_arr == phase)
    return t[mask], y[mask]


# --------------------------------------------------------------------------- #
#  Grace/dwell run-length classifier (timeseries criteria)
# --------------------------------------------------------------------------- #
def classify_excursions(t, y, lo, hi, grace_s):
    """Classify out-of-bounds excursions by their longest contiguous duration.

    Returns (status, worst_value, longest_run_s, n_runs).
      ok   - never out of bounds
      flag - some excursion(s) but the longest contiguous run <= grace_s (transient; never fails)
      fail - some run > grace_s (sustained); or any excursion at all when grace_s <= 0
    Run duration is edge-to-edge with a half-sample pad each side, so a lone out-of-bounds sample
    reads ~one dt (not 0 s) and is not silently tolerated by a grace > 0 test."""
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    valid = np.isfinite(t) & np.isfinite(y)
    if not valid.any():
        return "na", float("nan"), 0.0, 0
    oob = np.zeros(len(y), dtype=bool)
    if lo is not None:
        oob |= (y < lo)
    if hi is not None:
        oob |= (y > hi)
    oob &= valid
    worst = _worst_value(y[valid], lo, hi)
    status, longest, nruns = _classify_oob(t, oob, grace_s)
    return status, worst, longest, nruns


def _classify_oob(t, oob, grace_s):
    """Run-length grace classifier on a precomputed boolean out-of-bounds mask.
    Returns (status, longest_run_s, n_runs). Shared by threshold and envelope criteria."""
    oob = np.asarray(oob, dtype=bool)
    if not oob.any():
        return "ok", 0.0, 0
    idx = np.flatnonzero(oob)
    runs = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
    longest = 0.0
    for r in runs:
        i0, i1 = int(r[0]), int(r[-1])
        left_dt = (t[i0] - t[i0 - 1]) if i0 > 0 else (t[i0 + 1] - t[i0] if len(t) > 1 else 0.0)
        right_dt = (t[i1 + 1] - t[i1]) if i1 + 1 < len(t) else (t[i1] - t[i1 - 1] if len(t) > 1 else 0.0)
        dur = (t[i1] - t[i0]) + 0.5 * max(left_dt, 0.0) + 0.5 * max(right_dt, 0.0)
        longest = max(longest, dur)
    if grace_s is None or grace_s <= 0:
        return "fail", longest, len(runs)
    return ("flag" if longest <= float(grace_s) else "fail"), longest, len(runs)


# --------------------------------------------------------------------------- #
#  Flight-envelope (V, gamma) safe-band membership
# --------------------------------------------------------------------------- #
_ENV_CACHE = {}


def load_envelope(path):
    """Load a safe-airspeed-band CSV [gamma_deg, Vlo, Vhi] (NaN rows where no safe band exists).
    Returns (gamma, vlo, vhi) sorted by gamma, or None if unreadable. Cached by path."""
    if path in _ENV_CACHE:
        return _ENV_CACHE[path]
    env = None
    try:
        arr = np.genfromtxt(path, delimiter=",")
        if arr.ndim == 2 and arr.shape[1] >= 3:
            g = arr[:, 0]
            order = np.argsort(g)
            env = (g[order], arr[order, 1], arr[order, 2])
    except Exception:
        env = None
    _ENV_CACHE[path] = env
    return env


def envelope_oob(gamma, V, env):
    """Boolean out-of-band mask + worst exceedance [m/s] for each (gamma, V) sample.

    A sample is OUT if V < Vlo(gamma) or V > Vhi(gamma), or gamma is outside the band's valid
    range (interp returns NaN -> no safe flight there). Returns (oob, exceed_mps, valid)."""
    g, vlo, vhi = env
    ok = np.isfinite(vlo) & np.isfinite(vhi)
    if ok.sum() < 2:
        return None
    gv, lov, hiv = g[ok], vlo[ok], vhi[ok]
    lo_i = np.interp(gamma, gv, lov, left=np.nan, right=np.nan)   # clamp-to-NaN outside -> OUT
    hi_i = np.interp(gamma, gv, hiv, left=np.nan, right=np.nan)
    valid = np.isfinite(gamma) & np.isfinite(V)
    below = np.where(np.isfinite(lo_i), lo_i - V, 0.0)
    above = np.where(np.isfinite(hi_i), V - hi_i, 0.0)
    exceed = np.maximum(below, above)                            # >0 = m/s outside band; <=0 = inside (margin)
    oob = valid & ((~np.isfinite(lo_i)) | (~np.isfinite(hi_i)) | (V < lo_i) | (V > hi_i))
    return oob, exceed, valid


# --------------------------------------------------------------------------- #
#  Oscillation: per-phase control-command activity + spectrum
# --------------------------------------------------------------------------- #
def _detrend_lin(y):
    """Remove mean+slope (vectorized, no scipy dependency for the simple case)."""
    if len(y) < 2:
        return y - np.mean(y) if len(y) else y
    return y - np.linspace(y[0], y[-1], len(y))


def _reversal_rate(t, y):
    """Zero-crossings/sec of the detrended signal (oscillation proxy), with a 0.1-sigma deadband
    so noise-floor jitter is not counted. ~2 crossings per oscillation cycle."""
    if len(y) < 4:
        return float("nan")
    yd = _detrend_lin(y)
    sd = np.std(yd)
    T = float(t[-1] - t[0])
    if T <= 0:
        return float("nan")
    if sd <= 0:
        return 0.0
    s = np.sign(yd)
    s[np.abs(yd) < 0.1 * sd] = 0          # deadband -> ignore tiny wiggle
    s = s[s != 0]
    if len(s) < 2:
        return 0.0
    crossings = int(np.sum(np.abs(np.diff(s)) == 2))
    return crossings / T


def _band_fraction(t, y, cutoff_hz):
    """Fraction of PSD power above cutoff_hz (length-invariant), plus dominant frequency and fs.

    Interpolate onto a uniform grid at the median dt, linear-detrend, then Welch PSD (density) if
    scipy is available, else a Hann-windowed periodogram. Returns (band_frac, dom_hz, fs)."""
    if len(y) < 16:
        return float("nan"), float("nan"), float("nan")
    dt = float(np.median(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0:
        return float("nan"), float("nan"), float("nan")
    fs = 1.0 / dt
    tu = np.arange(t[0], t[-1], dt)
    if len(tu) < 16:
        return float("nan"), float("nan"), fs
    yu = np.interp(tu, t, y)
    if _HAVE_SCIPY:
        yd = _sp_detrend(yu, type="linear")
        f, pxx = _sp_welch(yd, fs=fs, window="hann", nperseg=min(len(yd), 256),
                           detrend="linear", scaling="density")
    else:
        yd = _detrend_lin(yu)
        w = np.hanning(len(yd))
        pxx = np.abs(np.fft.rfft(yd * w)) ** 2
        f = np.fft.rfftfreq(len(yd), dt)
    total = float(_TRAP(pxx, f))
    if total <= 0:
        return 0.0, float("nan"), fs
    above = f > cutoff_hz
    band = float(_TRAP(pxx[above], f[above]) / total) if above.any() else 0.0
    dom = float(f[1:][int(np.argmax(pxx[1:]))]) if len(pxx) > 1 else float("nan")
    return band, dom, fs


def _esc(val, flag, fail):
    """Escalate a single sub-metric to ok/flag/fail given its flag/fail thresholds (higher=worse)."""
    if val is None or not np.isfinite(val) or flag is None:
        return "ok"
    if fail is not None and val >= float(fail):
        return "fail"
    if val >= float(flag):
        return "flag"
    return "ok"


def _osc_channel_rows(df, phase_arr, sig, role, crit, case_id, key, slew_lim_all=None):
    """Per-phase oscillation metrics for ONE signal (band fraction, dominant freq, reversal rate,
    slew saturation, rms). Returns (worst_status, worst_value, rows); each row is tagged with `role`
    (command / actual / rate). slew_lim_all (the per-sample slew limit) is passed only for the
    command channel — slew saturation is a command-side concept."""
    t_all, y_all = get_signal(df, sig)
    if y_all is None:
        return "na", float("nan"), []
    spec_phases = set(crit.get("phases", []) or [])
    cutoff = float(crit.get("cutoff_hz", 1.0))
    min_n = int(crit.get("min_samples", 64))
    bf_flag, bf_fail = crit.get("band_frac_flag"), crit.get("band_frac_fail")
    rv_flag, rv_fail = crit.get("reversal_rate_flag"), crit.get("reversal_rate_fail")
    sl_flag, sl_fail = crit.get("slew_sat_flag"), crit.get("slew_sat_fail")
    slew_ratio = float(crit.get("slew_sat_ratio", 0.9))
    rms_min = crit.get("rms_min")

    rows = []
    worst_status, worst_val = "na", float("nan")
    for ph in pd.unique(phase_arr):
        mask = (phase_arr == ph)
        n = int(mask.sum())
        if n == 0:
            continue
        t, y = t_all[mask], y_all[mask]
        rms = float(np.std(_detrend_lin(y))) if n >= 2 else float("nan")
        rev = _reversal_rate(t, y)
        slew = float("nan")
        if slew_lim_all is not None and n >= 3:
            lim = slew_lim_all[mask]
            rate = np.abs(np.gradient(y, t))          # demanded command rate (deg/s); compare to limit
            good = np.isfinite(lim) & np.isfinite(rate) & (lim > 0)
            if good.any():
                slew = float(np.mean(rate[good] >= slew_ratio * lim[good]))
        feasible = (str(ph) in spec_phases) and (n >= min_n)
        band = dom = float("nan")
        if feasible:
            band, dom, _ = _band_fraction(t, y, cutoff)

        quiet = (rms_min is not None) and np.isfinite(rms) and (rms < float(rms_min))
        if quiet:
            status = "ok"
        else:
            subs = []
            if np.isfinite(band):
                subs.append(_esc(band, bf_flag, bf_fail))
            if np.isfinite(rev):
                subs.append(_esc(rev, rv_flag, rv_fail))
            if np.isfinite(slew):
                subs.append(_esc(slew, sl_flag, sl_fail))
            status = max(subs, key=lambda s: _RANK[s]) if subs else "na"

        drv = band if np.isfinite(band) else (rev if np.isfinite(rev) else slew)
        rows.append({"case_id": case_id, "criterion": key, "channel": sig, "role": role,
                     "phase": str(ph), "n_samples": n, "feasible": bool(feasible), "rms": rms,
                     "band_frac": band, "dom_hz": dom, "reversal_rate": rev,
                     "slew_sat": slew, "status": status})
        if _RANK[status] > _RANK[worst_status]:
            worst_status, worst_val = status, float(drv) if drv is not None else float("nan")
    return worst_status, worst_val, rows


def evaluate_oscillation(df, phase_arr, crit, case_id, key):
    """Per-phase control-oscillation check, per axis.

    The criterion VERDICT is driven by the COMMAND channel (`signal`) — the actionable signal (a
    controller fix). Optional `companions` (the actual surface deflection and the body rate) are
    analysed too and emitted as extra rows (role-tagged) for the command-vs-response diagnostic, but
    they do NOT change the verdict. Returns (rolled_up_status, worst_value, rows)."""
    slew_col = crit.get("slew_signal")               # per-sample slew-rate LIMIT (deg/s), command-side
    slew_lim_all = (df[slew_col].values.astype(float)
                    if slew_col and slew_col in df.columns else None)
    status, worst, rows = _osc_channel_rows(
        df, phase_arr, crit.get("signal"), "command", crit, case_id, key, slew_lim_all)
    for comp in (crit.get("companions", []) or []):  # actual surface + body rate (diagnostic only)
        _, _, crows = _osc_channel_rows(
            df, phase_arr, comp.get("signal"), comp.get("role", "response"), crit, case_id, key)
        rows.extend(crows)
    return status, worst, rows


# --------------------------------------------------------------------------- #
#  Per-case entry point
# --------------------------------------------------------------------------- #
def evaluate_case(df, fl, metrics, config, case_id):
    """Grade one case. Returns (fc_dict, osc_rows). ({}, []) when config is None.

    fc_dict has per-criterion fc_<key> (ok/flag/fail/na) + fc_<key>_val (worst value), plus summary
    fc_verdict / fc_n_failed / fc_n_flagged / fc_failed_list / fc_flagged_list / fc_phase_source.
    osc_rows is the long-format per-(channel,phase) oscillation detail."""
    if not config:
        return {}, []
    crits = config.get("criteria", {}) or {}
    phase_arr, src = build_phase_timeline(df, fl)

    out, osc_rows = {}, []
    failed, flagged = [], []
    crit_fail = degr_fail = False

    for key, c in crits.items():
        if key.startswith("_") or not isinstance(c, dict):
            continue                                   # comment / note keys (e.g. "_envelope_comment")
        kind = str(c.get("kind", "")).lower()
        sev = str(c.get("severity", "degraded")).lower()
        status, worst = "na", float("nan")
        run_s = None
        try:
            if kind == "scalar":
                lo, hi = _bounds(c)
                worst = _safe_float(metrics.get(c.get("metric")))
                status = _scalar_status(worst, lo, hi)
            elif kind == "timeseries":
                lo, hi = _bounds(c)
                t, y = get_signal(df, c.get("signal"))
                if y is None:
                    status = "na"
                else:
                    # optional gate: only judge samples where gate_signal is in [gate_min, gate_max]
                    # (e.g. gate AoA/elevator on TAS >= 6 m/s, so ground/near-zero-airspeed samples —
                    #  where AoA = atan2(.) is undefined ~180 deg — are excluded).
                    gsig = c.get("gate_signal")
                    if gsig:
                        gt, gy = get_signal(df, gsig)
                        if gy is not None and len(gy) == len(y):
                            keep = np.isfinite(gy)
                            if c.get("gate_min") is not None:
                                keep &= (gy >= float(c["gate_min"]))
                            if c.get("gate_max") is not None:
                                keep &= (gy <= float(c["gate_max"]))
                            y = np.where(keep, y, np.nan)        # gated-out samples ignored downstream
                    t, y = _slice_phase(t, y, phase_arr, c.get("phase"))
                    if len(y) == 0 or not np.isfinite(y).any():
                        status = "na"
                    else:
                        status, worst, run_s, _ = classify_excursions(
                            t, y, lo, hi, _safe_float(c.get("grace_s", 0.0)))
            elif kind == "oscillation":
                status, worst, rows = evaluate_oscillation(df, phase_arr, c, case_id, key)
                osc_rows.extend(rows)
            elif kind == "envelope":
                tV, V = get_signal(df, c.get("signal_v", "TAS_mps"))
                _,  Gm = get_signal(df, c.get("signal_gamma", "gamma_deg"))
                fname = c.get("envelope_file")
                env = None
                if fname:
                    cdir = config.get("_config_dir", "")
                    for p in (fname, os.path.join(cdir, fname) if cdir else fname):
                        env = load_envelope(p)
                        if env is not None:
                            break
                res = envelope_oob(Gm, V, env) if (V is not None and Gm is not None and env) else None
                if res is None:
                    status = "na"
                else:
                    oob, exceed, valid = res
                    gate_min = c.get("gate_min", 6.0)            # only judge airborne samples
                    if gate_min is not None:
                        keep = np.isfinite(V) & (V >= float(gate_min))
                        oob = oob & keep; valid = valid & keep
                    ph = c.get("phase")
                    if ph:
                        pm = (phase_arr == ph); oob = oob & pm; valid = valid & pm
                    if not valid.any():
                        status = "na"
                    else:
                        status, run_s, _ = _classify_oob(tV, oob, _safe_float(c.get("grace_s", 0.0)))
                        ex = exceed[valid]
                        worst = float(np.nanmax(ex)) if np.isfinite(ex).any() else float("nan")
            else:
                print(f"failure_criteria: WARNING criterion {key!r} has unknown kind {kind!r} — na")
                status = "na"
        except Exception as e:
            print(f"failure_criteria: WARNING criterion {key!r} errored ({e}) — na")
            status = "na"

        out[f"fc_{key}"] = status
        out[f"fc_{key}_val"] = float(worst) if np.isfinite(_safe_float(worst)) else np.nan
        if run_s is not None:
            out[f"fc_{key}_run_s"] = run_s
        if status == "fail":
            failed.append(key)
            if sev == "critical":
                crit_fail = True
            else:
                degr_fail = True
        elif status == "flag":
            flagged.append(key)

    out["fc_verdict"] = "FAIL" if crit_fail else ("PARTIAL" if degr_fail else "PASS")
    out["fc_n_failed"] = len(failed)
    out["fc_n_flagged"] = len(flagged)
    out["fc_failed_list"] = "|".join(failed)
    out["fc_flagged_list"] = "|".join(flagged)
    out["fc_phase_source"] = src
    return out, osc_rows
