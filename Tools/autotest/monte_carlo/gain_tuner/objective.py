"""Episode cost + robust aggregation.

Hierarchical penalty scalarization — three numerically non-overlapping layers
so survival dominates everything and critical breaches dominate soft
performance (lexicographic behavior in a single scalar CMA-ES can rank):

  L1  crash/timeout        j = 1000*(1 - progress)            in [~150, 1000]
  L2  landed + critical    j = 100 + 20*n_critical + j_soft   in [100, ~300]
  L3  clean flight         j = j_soft                         in [0, ~10]

j_soft = weighted blocks (engine-out recovery, doublet tracking, landing,
control health), every metric normalized by one hinge phi(x; T_pass, T_fail):
0.25 * x/T_pass below the pass threshold (credit for margin, 4x cheaper than a
breach) + a breach ramp capped at 2 (one blown metric can't be traded against
ten small wins). Thresholds seed from DASHBOARD/failure_criteria_config.json
limits; the *_calibratable ones should be re-seeded from a baseline batch
(median -> T_pass, p90 -> T_fail) and then FROZEN for the whole campaign.

Aggregation: J = CVaR_alpha(costs) + mean_w * mean(costs) over the CRN
evaluation set — tail-driven (robustness objective) with a small bulk anchor
so clean candidates still improve. Infrastructure-invalid episodes are
EXCLUDED before aggregation (never penalized: infra noise is not the gains'
fault and penalizing it corrupts the fitness landscape).
"""
import numpy as np

# ---- L1: graded progress by phase reached (plateau-free crash penalty) ----
_PROGRESS = {
    "crashed_climb":      0.15,
    "ground_strike_climb": 0.15,
    "climb_timeout":      0.20,
    "crashed_cruise":     0.40,   # includes doublet departures
    "ABORT_LOW_ALT":      0.45,
    "deadline":           0.55,
    "no_touchdown":       0.55,
    "crashed_approach":   0.65,
    "crashed_flare":      0.85,
}

# Exit reasons that are the INFRASTRUCTURE's fault, not the gains': the episode
# is retried once and then excluded from the aggregate.
INFRA_REASONS = {
    "sitl_port_never_opened", "mavlink_connect_failed", "fbwa_mode_failed",
    "arm_failed", "ust_unavailable", "ustf_unavailable",
}


def is_infra_failure(result: dict) -> bool:
    er = str(result.get("exit_reason", ""))
    return er in INFRA_REASONS or er.startswith(("exception", "worker_exception"))


def phi(x, t_pass, t_fail):
    """Hinge normalizer: sub-threshold slope 0.25, breach ramp capped at 2."""
    if x is None:
        return 0.0
    x = float(x)
    below = 0.25 * min(max(x / t_pass, 0.0), 1.0)
    breach = min(max((x - t_pass) / max(t_fail - t_pass, 1e-9), 0.0), 2.0)
    return below + breach


def band(x, lo, hi, half_width):
    """phi for band metrics: distance outside [lo, hi], T_pass=0+, T_fail=half_width."""
    if x is None:
        return 0.0
    d = max(lo - x, x - lo, 0.0) if lo == hi else max(lo - x, x - hi, 0.0)
    return phi(max(d, 0.0), 1e-9, half_width) if d > 0 else 0.0


# ---- thresholds (T_pass, T_fail). Sources: DASHBOARD failure_criteria_config
# (certified: td_sink 1.0, load 1.5g/2.5g doublet, ground_roll 15 m, roll
# tracking 8 deg, yaw peak 45 dps, oscillation 0.15/0.30 band + 2/4 Hz
# reversal) — others are placeholders to CALIBRATE from a baseline batch. ----
TH = {
    # engine-out recovery block
    "climb_yaw_rate_max_dps": (20.0, 45.0),
    "climb_roll_max_deg":     (15.0, 35.0),
    "track_dev_max_m":        (30.0, 80.0),      # calibratable
    "doublet_alt_sag_m":      (10.0, 30.0),      # calibratable
    # tracking block
    "roll_err_max_deg":       (8.0, 16.0),
    "roll_err_mean_deg":      (4.0, 8.0),
    "yaw_peak_rate_dps":      (45.0, 70.0),
    "yaw_bank_max_deg":       (15.0, 45.0),
    # landing block
    "td_sink_mps":            (1.0, 2.0),
    "flare_max_sink_mps":     (1.5, 2.5),
    "approach_max_sink_mps":  (2.5, 4.0),
    "ground_roll_m":          (15.0, 30.0),
    "landing_err_m":          (20.0, 60.0),      # calibratable
    "landing_roll_m":         (15.0, 30.0),
    # control-health block
    "dt_sat_frac":            (0.20, 0.50),
    "thr_sat_frac":           (0.30, 0.60),
    "roll_reversal_hz":       (2.0, 4.0),
    "yaw_reversal_hz":        (2.0, 4.0),
    "roll_band_frac":         (0.15, 0.30),
    "yaw_band_frac":          (0.15, 0.30),
}

_BLOCK_W = {"engineout": 0.40, "tracking": 0.30, "landing": 0.30, "ctrl": 0.15}


def _p(m, key):
    return phi(m.get(key), *TH[key])


def soft_cost(m: dict, fail_case: str) -> tuple:
    """(j_soft, block-cost dict) from an episode-metrics dict."""
    eo = np.mean([_p(m, "climb_yaw_rate_max_dps"), _p(m, "climb_roll_max_deg"),
                  _p(m, "track_dev_max_m"), _p(m, "doublet_alt_sag_m")])
    tr = np.mean([_p(m, "roll_err_max_deg"), _p(m, "roll_err_mean_deg"),
                  _p(m, "yaw_peak_rate_dps"), _p(m, "yaw_bank_max_deg"),
                  1.0 * (m.get("n_recover_events") or 0),
                  0.0 if m.get("doublets_flown") else 2.0])
    ld = np.mean([_p(m, "td_sink_mps"), _p(m, "flare_max_sink_mps"),
                  _p(m, "approach_max_sink_mps"), _p(m, "ground_roll_m"),
                  _p(m, "landing_err_m"), _p(m, "landing_roll_m"),
                  band(m.get("td_pitch_deg"), 0.0, 12.0, 6.0),
                  band(m.get("td_speed_mps"), 10.0, 15.0, 3.0)])
    ct = np.mean([_p(m, "dt_sat_frac"), _p(m, "thr_sat_frac"),
                  _p(m, "roll_reversal_hz"), _p(m, "yaw_reversal_hz"),
                  _p(m, "roll_band_frac"), _p(m, "yaw_band_frac")])
    w = dict(_BLOCK_W)
    if fail_case == "none":
        # Healthy episode: no engine-out block; renormalize the remaining
        # objective blocks to the same total weight (0.7) so healthy and
        # failure episodes are cost-commensurate.
        w["engineout"] = 0.0
        s = w["tracking"] + w["landing"]
        w["tracking"] = w["tracking"] / s * (_BLOCK_W["engineout"] + s)
        w["landing"]  = w["landing"] / s * (_BLOCK_W["engineout"] + s)
    blocks = {"engineout": float(eo), "tracking": float(tr),
              "landing": float(ld), "ctrl": float(ct)}
    j = sum(w[k] * blocks[k] for k in blocks)
    return float(j), blocks


def critical_breaches(m: dict, result: dict) -> list:
    """The non-negotiable (DASHBOARD-critical) limits: any breach -> L2."""
    out = []
    if (m.get("td_sink_mps") or 0.0) > 1.0:
        out.append("td_sink>1.0")
    lf = m.get("max_load_factor_total")
    if lf is not None and lf > 2.5:                 # doublet g-limit (worst case)
        out.append("load>2.5g")
    nz = m.get("min_load_factor_nz")
    if nz is not None and nz < -1.0:
        out.append("nz<-1.0")
    if (m.get("ground_roll_m") or 0.0) > 15.0:
        out.append("ground_roll>15")
    if (m.get("n_recover_events") or 0) >= 2:
        out.append("repeated_departures")
    return out


def episode_cost(metrics: dict, result: dict) -> dict:
    """-> {cost, layer, progress|n_critical, blocks} for one VALID episode."""
    lr = str(result.get("land_result", result.get("exit_reason", "unknown")))
    if lr != "landed":
        progress = _PROGRESS.get(lr, 0.10)
        # continuous refinement: longer survival within a failed episode still
        # counts a little (t vs the 420 s watchdog), so equal-phase crashers
        # remain distinguishable.
        progress += 0.10 * min(float(result.get("duration_s", 0.0)) / 420.0, 1.0)
        return {"cost": 1000.0 * (1.0 - min(progress, 0.99)), "layer": "L1",
                "progress": progress, "blocks": None}
    j_soft, blocks = soft_cost(metrics, result.get("fail_case", "none"))
    crits = critical_breaches(metrics, result)
    if crits:
        return {"cost": 100.0 + 20.0 * len(crits) + j_soft, "layer": "L2",
                "criticals": crits, "blocks": blocks}
    return {"cost": j_soft, "layer": "L3", "blocks": blocks}


def aggregate(costs, alpha: float = 0.25, mean_w: float = 0.25) -> dict:
    """Robust fitness over the eval set: CVaR_alpha + mean_w * mean.
    `costs` = valid episode costs only (infra-invalid already excluded)."""
    a = np.asarray([c for c in costs if c is not None], dtype=float)
    if len(a) == 0:
        return {"fitness": None, "cvar": None, "mean": None, "n": 0}
    k = max(1, int(round(alpha * len(a))))
    tail = np.sort(a)[-k:]
    cvar = float(np.mean(tail))
    mean = float(np.mean(a))
    return {"fitness": cvar + mean_w * mean, "cvar": cvar, "mean": mean,
            "worst": float(np.max(a)), "n": int(len(a))}
