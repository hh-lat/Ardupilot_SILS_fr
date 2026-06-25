#!/usr/bin/env python3
"""
build_dashboard_data.py [results_dir]

Combine a Monte Carlo campaign into the dashboard inputs:
  dashboard_data.xlsx / .csv  - ONE row per case: outcomes + computed performance
                                metrics + all perturbed parameters (the "Excel").
  timeseries.parquet          - downsampled per-case time series (long format,
                                keyed by case_id) for the drill-down view.

Summary source (auto-detected):
  * NEW campaigns: per-case  case_*/result.json   (no top-level summary.csv).
    Parameters there are stored UN-prefixed (mass, prop_CT_1, ...); they are
    re-prefixed to p_<name> to match the dashboard's `p_*` convention.
  * OLD campaigns: top-level summary.csv (params already p_-prefixed).

Per-case time series is read from sim_output_*.parquet when present (new runners),
else sim_output_*.csv (older campaigns). Cases whose sim_output is missing still
appear (outcomes + params from the summary), just with NaN computed metrics and no
drill-down time series.

Run after a campaign, then commit these two files to the dashboard repo for Render.
Usage:  python3 build_dashboard_data.py /path/to/mc_20260618_171051
"""
import glob, json, math, os, sys
import numpy as np
import pandas as pd

import failure_criteria as fc

RESULTS   = sys.argv[1] if len(sys.argv) > 1 else "../mc_results_20260611_173053"
OUTDIR    = os.path.dirname(os.path.abspath(__file__))
CRIT_CFG, CRIT_KEYS = fc.load_criteria_config(OUTDIR)   # (None, []) if no config -> feature off
G         = 9.81
TS_POINTS = 150     # ~points per case in the downsampled time series
TS_COLS   = ["Time_s", "alt_agl_m", "TAS_mps", "phi", "theta", "psi", "p", "q", "r",
             "V_ned_gnd_0", "V_ned_gnd_1", "V_ned_gnd_2", "V_b_tas_0", "V_b_tas_1",
             "delta_e", "delta_aL", "delta_aR", "delta_a", "delta_r",
             "mot0_thr_cmd", "Lift_N", "Drag_N", "total_rotor_force"]


def load_summary(results):
    """One row per case: outcome fields + perturbed params (p_-prefixed).

    Prefers per-case case_*/result.json (new campaigns). Falls back to a
    top-level summary.csv (older campaigns)."""
    rfiles = sorted(glob.glob(os.path.join(results, "case_*", "result.json")))
    if rfiles:
        rows = []
        for f in rfiles:
            with open(f) as fh:
                d = json.load(fh)
            params = d.pop("params", {}) or {}
            row = dict(d)                                   # outcome / metric fields
            row.update({f"p_{k}": v for k, v in params.items()})
            rows.append(row)
        summ = pd.DataFrame(rows)
        if "case_id" not in summ.columns:
            sys.exit("result.json files have no 'case_id' field")
        summ["case_id"] = summ["case_id"].astype(int)
        summ = summ.set_index("case_id").sort_index()
        n_par = sum(c.startswith("p_") for c in summ.columns)
        print(f"summary: {summ.shape[0]} cases from result.json "
              f"({summ.shape[1]} cols, {n_par} params)")
        return summ
    csv = os.path.join(results, "summary.csv")
    if os.path.exists(csv):
        summ = pd.read_csv(csv).set_index("case_id")
        print(f"summary: {summ.shape[0]} cases from summary.csv, {summ.shape[1]} cols")
        return summ
    sys.exit(f"no case_*/result.json or summary.csv found under {results}")


def write_param_nominals(results, outdir):
    """Emit param_nominals.csv (param, nominal, sigma, description) from the MC config
    so the dashboard can show every parameter's baseline + how extreme each draw is.
    Searches the campaign dir, the monte_carlo dir (parent of DASHBOARD) and OUTDIR."""
    cands = (glob.glob(os.path.join(results, "monte_carlo_config*.json")) +
             glob.glob(os.path.join(outdir, "..", "monte_carlo_config_ustol_v1.json")) +
             glob.glob(os.path.join(outdir, "..", "monte_carlo_config*.json")) +
             glob.glob(os.path.join(outdir, "monte_carlo_config*.json")))
    for cfg_path in cands:
        try:
            cfg = json.load(open(cfg_path))
            params = cfg["params"]
        except Exception:
            continue
        smul = cfg.get("sigma_multiplier", 3) or 3
        rows = [{"param": f"p_{k}", "nominal": v.get("nominal"),
                 "sigma": (v["sigma_3"] / smul) if v.get("sigma_3") is not None else None,
                 "description": v.get("description", "")}
                for k, v in params.items()]
        pd.DataFrame(rows).to_csv(os.path.join(outdir, "param_nominals.csv"), index=False)
        print(f"wrote param_nominals.csv          : {len(rows)} params (from {os.path.basename(cfg_path)})")
        return
    print("param_nominals.csv: no monte_carlo_config*.json found — dashboard will "
          "fall back to campaign median/std for nominals")


def landing_metrics(fl, sim_df):
    """Touchdown / approach landing metrics from the phase-labelled flight.csv
    (APPROACH -> FLARE -> ROLLOUT -> GROUND), plus a kinematic load-factor estimate
    from the sim_output sink rate. Returns {} for runs that never landed.
    `fl` is the already-loaded flight DataFrame (or None) — read once by per_case."""
    if fl is None or "phase" not in fl.columns or len(fl) < 3:
        return {}
    flare = fl[fl["phase"] == "FLARE"]
    appr  = fl[fl["phase"] == "APPROACH"]
    roll  = fl[fl["phase"] == "ROLLOUT"]
    if len(flare):                       # touchdown = last airborne (FLARE) sample
        td = flare.iloc[-1]
    elif len(roll):                      # fall back to first ROLLOUT sample
        td = roll.iloc[0]
    else:
        return {}                        # never reached the flare/touchdown sequence
    m = {
        "td_speed_mps":          float(td["airspeed"]),
        "td_pitch_deg":          float(td["pitch_deg"]),
        "td_sink_mps":           float(-td["climb_mps"]),
        "approach_max_sink_mps": float(-appr["climb_mps"].min()) if len(appr) else np.nan,
        "approach_speed_mps":    float(appr["airspeed"].mean()) if len(appr) else np.nan,
        "flare_max_sink_mps":    float(-flare["climb_mps"].min()) if len(flare) else np.nan,
        "flare_duration_s":      float(flare["time_s"].max() - flare["time_s"].min())
                                 if len(flare) else np.nan,
    }
    # Kinematic vertical load-factor estimate near touchdown: n_z = 1 + a_up/g,
    # a_up = -d(V_ned_down)/dt. Rough (10 Hz logging, no modeled impact spike).
    try:
        t  = sim_df["Time_s"].values
        a_up = -np.gradient(sim_df["V_ned_gnd_2"].values, t)
        nz = 1.0 + a_up / G
        # flight.csv time and sim_output Time_s differ by ~20 s (pre-arm); align td time to sim clock
        td_t = float(td["time_s"]) + fc._flight_time_offset(sim_df, fl)
        w = (t >= td_t - 2.0) & (t <= td_t + 0.5)
        m["td_load_factor_est"] = float(np.nanmax(nz[w])) if w.any() else np.nan
    except Exception:
        m["td_load_factor_est"] = np.nan
    return m


def per_case(cdir, mass, cid, logged=None):
    # Prefer Parquet (new runners write sim_output_*.parquet — far smaller / faster to
    # load); fall back to CSV (older campaigns). Switching the runner output "just works".
    fs = (sorted(glob.glob(os.path.join(cdir, "sim_output_*.parquet"))) or
          sorted(glob.glob(os.path.join(cdir, "sim_output_*.csv"))))
    if not fs:
        return None, None, []
    df = pd.read_parquet(fs[0]) if fs[0].endswith(".parquet") else pd.read_csv(fs[0])
    if len(df) < 5:
        return None, None, []
    # derived channels
    df["alpha_deg"] = np.degrees(np.arctan2(df["V_b_tas_2"], df["V_b_tas_0"]))
    df["gamma_deg"] = np.degrees(np.arctan2(-df["V_ned_gnd_2"],
                                            np.hypot(df["V_ned_gnd_0"], df["V_ned_gnd_1"])))
    W = mass * G
    df["L_over_Wcosg"] = df["Lift_N"] / (W * np.cos(np.radians(df["gamma_deg"])))
    gs = np.hypot(df["V_ned_gnd_0"], df["V_ned_gnd_1"]).values
    st = df["plane_moving_state"].values
    t  = df["Time_s"].values

    m = {}
    i0 = next((i for i, s in enumerate(st) if s >= 1), None)   # roll start (first motion)
    # Ground roll ends at LIFT-OFF (wheels leave the ground = lift>=weight = lift-off speed reached).
    # The sim's airborne flag plane_moving_state==3 marks that instant (AGL just starts rising from 0).
    # Fall back to the first AGL>0.1 m sample if the flag is absent. NOTE: this is the roll to
    # LIFT-OFF — NOT to a screen height; the runner's result.json integrates to AGL>0.5 m, which adds
    # ~1 s of initial climb (~10-15 m) and so reads longer.
    iL = next((i for i, s in enumerate(st) if s == 3), None)
    if iL is None and i0 is not None:
        after = np.flatnonzero(df["alt_agl_m"].values[i0:] > 0.1)
        if len(after):
            iL = i0 + int(after[0])
    if i0 is not None and iL is not None and iL > i0:
        _trap = getattr(np, "trapezoid", getattr(np, "trapz", None))
        m["ground_roll_m"] = float(_trap(gs[i0:iL+1], t[i0:iL+1]))
        m["Vlof_mps"]      = float(df["TAS_mps"].iloc[iL])
        m["t_liftoff_s"]   = float(t[iL] - t[i0])
    else:
        m["ground_roll_m"] = m["Vlof_mps"] = m["t_liftoff_s"] = np.nan
    air = df["alt_agl_m"] > 2
    m["max_TAS_mps"]      = float(df["TAS_mps"].max())
    m["max_abs_p_dps"]    = float(np.degrees(df["p"].abs().max()))
    m["max_abs_q_dps"]    = float(np.degrees(df["q"].abs().max()))
    m["max_abs_r_dps"]    = float(np.degrees(df["r"].abs().max()))
    m["max_AoA_deg"]      = float(df["alpha_deg"].max())
    m["min_L_over_Wcosg"] = float(df.loc[air, "L_over_Wcosg"].min()) if air.any() else np.nan
    m["max_throttle"]     = float(df["mot0_thr_cmd"].max())

    # Read the phase-labelled flight log once; share it with landing_metrics and the criteria engine.
    fcsv = glob.glob(os.path.join(cdir, "*_flight.csv"))
    fl = pd.read_csv(fcsv[0]) if fcsv else None
    m.update(landing_metrics(fl, df))     # touchdown speed/pitch/sink, load-factor est.

    # Failure criteria (no-op when no config): adds fc_* keys + per-phase oscillation rows.
    # Scalar criteria can also reference fields the SIM logged directly into result.json/summary
    # (e.g. max_load_factor_total) — pass those alongside the computed metrics (computed wins on overlap).
    metrics_for_fc = {**(logged or {}), **m}
    fcd, osc = fc.evaluate_case(df, fl, metrics_for_fc, CRIT_CFG, cid)
    m.update(fcd)

    # Per-row flight phase (aligned to the sim clock) so the drill-down can shade phase regions.
    phase_arr, _ = fc.build_phase_timeline(df, fl)

    step = max(1, len(df) // TS_POINTS)
    ts = df.iloc[::step][[c for c in TS_COLS if c in df.columns]].copy()
    ts["alpha_deg"] = df["alpha_deg"].iloc[::step].values
    ts["gamma_deg"] = df["gamma_deg"].iloc[::step].values
    ts["phase"]     = np.asarray(phase_arr, dtype=object)[::step]
    # Derived display channels for the drill-down panels (computed on the downsampled frame).
    _deg = lambda c: np.degrees(ts[c]) if c in ts else np.nan
    if "phi" in ts:   ts["roll_deg"]  = _deg("phi")
    if "theta" in ts: ts["pitch_deg"] = _deg("theta")
    if "psi" in ts:   ts["yaw_deg"]   = _deg("psi")
    if "p" in ts:     ts["p_dps"]     = _deg("p")
    if "q" in ts:     ts["q_dps"]     = _deg("q")
    if "r" in ts:     ts["r_dps"]     = _deg("r")
    if "V_ned_gnd_0" in ts and "V_ned_gnd_1" in ts:
        ts["ground_speed_mps"] = np.hypot(ts["V_ned_gnd_0"], ts["V_ned_gnd_1"])
    if "V_ned_gnd_2" in ts: ts["climb_mps"] = -ts["V_ned_gnd_2"]
    if "V_b_tas_0" in ts and "V_b_tas_1" in ts:
        ts["beta_deg"] = np.degrees(np.arctan2(ts["V_b_tas_1"], ts["V_b_tas_0"]))
    if "mot0_thr_cmd" in ts: ts["throttle_pct"] = ts["mot0_thr_cmd"] * 100.0
    if "delta_e" in ts:  ts["elevator_deg"] = _deg("delta_e")
    if "delta_r" in ts:  ts["rudder_deg"]   = _deg("delta_r")
    ail = "delta_a" if "delta_a" in ts else ("delta_aL" if "delta_aL" in ts else None)
    if ail:              ts["aileron_deg"]  = np.degrees(ts[ail])
    return m, ts, osc


summ = load_summary(RESULTS)

rows, ts_all, osc_all = [], [], []
cdirs = sorted(glob.glob(os.path.join(RESULTS, "case_*")))
n_nosim = 0
for n, cdir in enumerate(cdirs):
    try:
        cid = int(os.path.basename(cdir).split("_")[1])
    except ValueError:
        continue
    if cid not in summ.index:
        continue
    mass = float(summ.loc[cid, "p_mass"]) if "p_mass" in summ.columns else 17.0
    logged = summ.loc[cid].to_dict()      # result.json/summary scalars (incl. logged load factor)
    m, ts, osc = per_case(cdir, mass, cid, logged)
    if m is None:
        n_nosim += 1
        continue
    m["case_id"] = cid
    rows.append(m)
    ts["case_id"] = cid
    ts_all.append(ts)
    if osc:
        osc_all.extend(osc)
    if n % 200 == 0:
        print(f"  ...{n}/{len(cdirs)}")

metrics  = pd.DataFrame(rows).set_index("case_id")
# Columns present in BOTH the summary and the computed metrics (e.g. ground_roll_m,
# which the new result.json also reports): prefer the computed value, fall back to
# the summary's where the sim_output CSV was missing.
dup = [c for c in metrics.columns if c in summ.columns]
combined = summ.join(metrics, rsuffix="_calc")
for c in dup:
    calc = combined[f"{c}_calc"]
    combined[c] = calc.where(calc.notna(), combined[c])
    combined = combined.drop(columns=[f"{c}_calc"])

# ---- Wind condition columns (present only for wind campaigns, e.g. aws_monte_carlo_7) ----
# result.json carries wind_case / wind_speed_mps / wind_dir_deg / wind_dir_z_deg. Normalise
# them and derive a convenient wind_on flag (wind active = a named type AND non-zero speed).
WIND_COLS = ["wind_case", "wind_on", "wind_speed_mps", "wind_dir_deg", "wind_dir_z_deg"]
if "wind_case" in combined.columns:
    combined["wind_case"] = combined["wind_case"].fillna("none").astype(str).str.lower()
    wspd = pd.to_numeric(combined.get("wind_speed_mps"), errors="coerce").fillna(0.0)
    combined["wind_speed_mps"] = wspd
    combined["wind_on"] = (combined["wind_case"] != "none") & (wspd > 0)
wind_cols = [c for c in WIND_COLS if c in combined.columns]

# put wind condition + computed metrics right after the outcome columns, then any extra
# summary fields (liftoff_speed, max_pitch_deg, time_to_*, ...), params last
outcome  = ["takeoff_success", "mission_complete", "phase_reached", "land_result",
            "max_alt_m", "duration_s", "exit_reason"]
param_cols  = [c for c in combined.columns if c.startswith("p_")]
metric_cols = [c for c in metrics.columns if c not in combined.index.names]
fc_cols     = [c for c in metric_cols if c.startswith("fc_")]
metric_cols = [c for c in metric_cols if not c.startswith("fc_")]
known   = set(outcome) | set(metric_cols) | set(param_cols) | set(wind_cols) | set(fc_cols)
extra   = [c for c in combined.columns if c not in known and not c.startswith("p_")]
ordered = [c for c in outcome if c in combined.columns] + wind_cols + \
          [c for c in metric_cols if c not in outcome] + fc_cols + extra + param_cols
combined = combined[ordered]

combined.to_csv(os.path.join(OUTDIR, "dashboard_data.csv"))
combined.to_excel(os.path.join(OUTDIR, "dashboard_data.xlsx"))
ts_df = pd.concat(ts_all, ignore_index=True)
ts_df.to_parquet(os.path.join(OUTDIR, "timeseries.parquet"), index=False)
# Per-phase control-oscillation detail (one row per case x channel x phase) for the dashboard's
# Failure-criteria tab. Written only when criteria produced oscillation rows.
osc_path = os.path.join(OUTDIR, "oscillation.parquet")
if osc_all:
    pd.DataFrame(osc_all).to_parquet(osc_path, index=False)
    print(f"wrote oscillation.parquet        : {len(osc_all)} rows "
          f"({pd.DataFrame(osc_all)['case_id'].nunique()} cases)")
elif os.path.exists(osc_path):
    os.remove(osc_path)   # stale file from a prior criteria run -> drop so the app won't read it
write_param_nominals(RESULTS, OUTDIR)

print(f"\nwrote dashboard_data.csv / .xlsx : {combined.shape[0]} cases x {combined.shape[1]} cols "
      f"({n_nosim} had no sim_output -> NaN metrics)")
print(f"wrote timeseries.parquet         : {len(ts_df)} rows ({ts_df['case_id'].nunique()} cases)")
print("metric columns:", metric_cols)
print("extra summary columns kept:", extra)
if "wind_case" in combined.columns:
    print("wind cases:", combined["wind_case"].value_counts().to_dict(),
          "| wind_on:", int(combined["wind_on"].sum()))
if "fc_verdict" in combined.columns:
    print("failure-criteria verdict:", combined["fc_verdict"].value_counts().to_dict())
