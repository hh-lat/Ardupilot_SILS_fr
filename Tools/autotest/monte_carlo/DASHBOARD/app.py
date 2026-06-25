"""
uSTOL_v1 Monte Carlo dashboard.

Explore a Monte Carlo dispersion study of an ArduPlane SITL blown-wing
ultra-STOL. Reads the files produced by build_dashboard_data.py (keep them next
to this app):
    dashboard_data.csv     - 1 row/case: outcomes + perf metrics + params
    timeseries.parquet     - downsampled per-case time series (drill-down)
    oscillation.parquet    - per-(case,channel,phase) control-oscillation detail (optional)

Run:  streamlit run app.py    ->  http://localhost:8501

Primary pass/fail label is `takeoff_success` (reached >=15 m AGL). `land_result`
is misleading (it marks never-flew cases as "landed"), so it is NOT used for
success accounting.
"""
import base64
import copy
import json
from collections import Counter
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
import plotly.io as pio
from plotly.subplots import make_subplots
import streamlit as st
from scipy.ndimage import gaussian_filter1d
from scipy.stats import gaussian_kde

st.set_page_config(page_title="uSTOL_v1 Monte Carlo · LAT Aerospace",
                   page_icon="✈", layout="wide")

NOMINAL = {            # nominal / reference values worth annotating
    "p_prop_CT_1": 0.692,
    "p_mass": 17.0,
}
CT1_FAILURE_THRESHOLD = 0.55   # failures cluster below this (see context doc)
TAKEOFF_ALT_M = 15.0   # success = peak AGL reached this (recomputed from data; see load_cases)

# --------------------------------------------------------------------------- #
#  LAT Aerospace theme — palette, fonts, custom CSS, Plotly template
# --------------------------------------------------------------------------- #
ASSETS   = Path(__file__).parent / "assets"
INK      = "#0a0e17"   # app background (near-black navy)
PANEL    = "#0f1726"   # cards / sidebar panels
LINE     = "rgba(255,255,255,0.07)"
TEXT     = "#e6eaf2"
MUTED    = "#7c8aa5"   # uppercase section labels
BLUE     = "#3b82f6"   # primary accent
CRIMSON  = "#e0334b"   # active tab underline / alerts
GREEN    = "#4ade80"   # status / success

SUCCESS_COLOR = GREEN
PARTIAL_COLOR = "#f59e0b"   # amber — took off but never landed
FAILURE_COLOR = "#f05a5a"

# Three-tier flight outcome (ordinal: No takeoff < Took off, no landing < Landed)
OUTCOME_SUCCESS = "Landed"                 # full success — touchdown reached
OUTCOME_PARTIAL = "Took off, no landing"   # partial — reached >=15 m AGL but never landed
OUTCOME_FAIL    = "No takeoff"             # failure — never reached >=15 m AGL
OUTCOME_ORDER   = [OUTCOME_SUCCESS, OUTCOME_PARTIAL, OUTCOME_FAIL]
OUTCOME_COLORS  = {OUTCOME_SUCCESS: SUCCESS_COLOR,
                   OUTCOME_PARTIAL: PARTIAL_COLOR,
                   OUTCOME_FAIL:    FAILURE_COLOR}
OUTCOME_RANK    = {OUTCOME_FAIL: 0, OUTCOME_PARTIAL: 1, OUTCOME_SUCCESS: 2}


def _logo_b64():
    f = ASSETS / "lat-logo.png"
    return base64.b64encode(f.read_bytes()).decode() if f.exists() else ""


def inject_theme():
    pio.templates["lat"] = copy.deepcopy(pio.templates["plotly_dark"])
    t = pio.templates["lat"].layout
    t.paper_bgcolor = "rgba(0,0,0,0)"
    t.plot_bgcolor  = "rgba(0,0,0,0)"
    t.font.family   = "Manrope, sans-serif"
    t.font.color    = "#c7d0e0"
    t.colorway      = [BLUE, GREEN, "#f59e0b", "#a78bfa", "#22d3ee", FAILURE_COLOR]
    for ax in (t.xaxis, t.yaxis):
        ax.gridcolor = "rgba(255,255,255,0.06)"
        ax.zerolinecolor = "rgba(255,255,255,0.10)"
        ax.linecolor = "rgba(255,255,255,0.15)"
    pio.templates.default = "lat"

    st.markdown(f"""
    <style>
    @import url('https://fonts.googleapis.com/css2?family=Lexend+Mega:wght@300;400;500;600&family=Manrope:wght@300;400;500;600;700;800&family=JetBrains+Mono:wght@400;500&display=swap');

    html, body, [class*="css"], .stApp {{ font-family: 'Manrope', sans-serif; }}
    .stApp {{
        background:
          radial-gradient(1100px 520px at 50% -8%, #14203a 0%, rgba(10,14,23,0) 60%),
          {INK};
    }}
    h1, h2, h3, h4 {{
        font-family: 'Manrope', sans-serif !important;
        font-weight: 700; letter-spacing: -.01em; color: {TEXT};
    }}

    /* ---- sidebar ---- */
    [data-testid="stSidebar"] {{
        background: {PANEL};
        border-right: 1px solid {LINE};
    }}
    [data-testid="stSidebar"]::before {{
        content: ""; position: absolute; top: 0; left: 0; right: 0; height: 3px;
        background: linear-gradient(90deg, {BLUE}, #7c5cff 60%, transparent);
    }}
    .lat-brand {{ padding: .25rem 0 1rem; }}
    .lat-brand img {{ height: 46px; margin-bottom: .55rem; }}
    .lat-eyebrow {{
        font-family: 'Lexend Mega', sans-serif;
        color: {BLUE}; font-size: .6rem; font-weight: 500;
        letter-spacing: .18em; text-transform: uppercase;
    }}
    .lat-brand .title {{
        font-family: 'Manrope', sans-serif; font-size: 1.4rem; font-weight: 700;
        color: {TEXT}; line-height: 1.15; margin-top: .25rem;
    }}

    /* uppercase tracked section labels (sidebar headers) */
    [data-testid="stSidebar"] h2, [data-testid="stSidebar"] h3 {{
        font-family: 'Lexend Mega', sans-serif !important;
        font-size: .62rem !important; font-weight: 500; color: {MUTED};
        letter-spacing: .14em; text-transform: uppercase;
    }}

    /* ---- tabs ---- */
    .stTabs [data-baseweb="tab-list"] {{
        gap: 2.2rem; border-bottom: 1px solid {LINE};
    }}
    .stTabs [data-baseweb="tab"] {{
        font-size: .82rem; font-weight: 600; letter-spacing: .12em;
        text-transform: uppercase; color: {MUTED}; padding: .4rem 0;
    }}
    .stTabs [aria-selected="true"] {{ color: {TEXT}; }}
    .stTabs [data-baseweb="tab-highlight"] {{ background-color: {CRIMSON}; height: 2px; }}

    /* ---- KPI metric cards ---- */
    [data-testid="stMetric"] {{
        background: {PANEL}; border: 1px solid {LINE}; border-radius: 12px;
        padding: .9rem 1.1rem;
    }}
    [data-testid="stMetricLabel"] p {{
        font-family: 'Lexend Mega', sans-serif;
        color: {MUTED}; text-transform: uppercase; letter-spacing: .08em;
        font-size: .58rem; font-weight: 500;
    }}
    [data-testid="stMetricValue"] {{ font-family: 'Manrope', sans-serif; font-weight: 700; }}

    /* main-header eyebrow + rule */
    .lat-eyebrow-main {{
        font-family: 'Lexend Mega', sans-serif;
        color: {BLUE}; font-size: .64rem; font-weight: 500;
        letter-spacing: .18em; text-transform: uppercase;
        border-bottom: 2px solid {BLUE}; display: inline-block; padding-bottom: .25rem;
    }}
    code, .stCode, [data-testid="stTable"] code {{ font-family: 'JetBrains Mono', monospace; }}
    [data-testid="stSidebar"] .stSuccess {{ color: {GREEN}; }}
    </style>
    """, unsafe_allow_html=True)


def sidebar_brand():
    logo = _logo_b64()
    img = f'<img src="data:image/png;base64,{logo}"/>' if logo else ""
    st.sidebar.markdown(
        f'<div class="lat-brand">{img}'
        f'<div class="lat-eyebrow">LAT Aerospace</div>'
        f'<div class="title">Monte&nbsp;Carlo Visualizer</div></div>',
        unsafe_allow_html=True)


inject_theme()

# --------------------------------------------------------------------------- #
#  Data loading (cached)
# --------------------------------------------------------------------------- #
@st.cache_data
def load_cases():
    df = pd.read_csv("dashboard_data.csv")
    if "mission_complete" in df:
        df["mission_complete"] = df["mission_complete"].astype(str).str.lower().eq("true")
    # The runner's raw `takeoff_success` only means "liftoff detected (live AGL >= 0.5 m)"
    # and is unreliable, so we derive robust, self-consistent labels from the data.
    if "takeoff_success" in df:
        df["liftoff_detected"] = df["takeoff_success"].astype(str).str.lower().eq("true")

    # Two physical facts straight from the data:
    took_off = (df["max_alt_m"] >= TAKEOFF_ALT_M).fillna(False)             # reached >=15 m AGL
    landed   = (df.get("land_result", pd.Series("", index=df.index))
                  .astype(str).str.strip().str.lower().eq("landed"))        # touchdown reached
    df["took_off"]        = took_off
    df["landed"]          = landed
    df["takeoff_success"] = took_off          # binary "did it fly?" used by the TV analysis

    # Three-tier outcome: Landed (full success) > Took off, no landing (partial) > No takeoff
    df["outcome"] = np.select([landed, took_off & ~landed],
                              [OUTCOME_SUCCESS, OUTCOME_PARTIAL],
                              default=OUTCOME_FAIL)
    df["outcome_rank"] = df["outcome"].map(OUTCOME_RANK).astype(int)

    # ---- Wind condition (present only for wind campaigns; recomputed robustly here) ----
    if "wind_case" in df.columns:
        df["wind_case"] = df["wind_case"].fillna("none").astype(str).str.lower()
        df["wind_speed_mps"] = (pd.to_numeric(df.get("wind_speed_mps"), errors="coerce")
                                  .fillna(0.0))
        for c in ("wind_dir_deg", "wind_dir_z_deg"):
            if c in df.columns:
                df[c] = pd.to_numeric(df[c], errors="coerce")
        df["wind_on"] = (df["wind_case"] != "none") & (df["wind_speed_mps"] > 0)

    # ---- Failure-criteria verdicts (present only when a criteria config was used) ----
    if "fc_verdict" in df.columns:
        df["criteria_verdict"] = df["fc_verdict"].fillna("PASS").astype(str)
        mission_pf = df["outcome"].map({OUTCOME_SUCCESS: "PASS",
                                        OUTCOME_PARTIAL: "PARTIAL",
                                        OUTCOME_FAIL:    "FAIL"})
        vrank = {"PASS": 2, "PARTIAL": 1, "FAIL": 0}
        # final = worse (lowest rank) of mission outcome and criteria verdict
        cv_r = df["criteria_verdict"].map(vrank).fillna(2)
        mo_r = mission_pf.map(vrank).fillna(2)
        df["final_verdict"] = np.where(cv_r <= mo_r, df["criteria_verdict"], mission_pf)
    return df

@st.cache_data
def load_ts():
    return pd.read_parquet("timeseries.parquet")

@st.cache_data
def load_param_meta(_df, params):
    """Per-parameter baseline: {param: (nominal, sigma, description)}.

    Prefers param_nominals.csv (written by build_dashboard_data.py from the MC config —
    the true nominal + 1σ). Falls back to the campaign's own median (≈ nominal) and std
    (≈ sigma) so the table still works if the config file is absent."""
    meta = {}
    f = Path("param_nominals.csv")
    if f.exists():
        m = pd.read_csv(f).set_index("param")
        for p in params:
            if p in m.index:
                r = m.loc[p]
                meta[p] = (float(r["nominal"]), float(r["sigma"]),
                           str(r.get("description", "")))
    for p in params:                       # fill any gaps from the data itself
        if p not in meta:
            col = _df[p].astype(float)
            meta[p] = (float(col.median()), float(col.std()), "(from campaign median/std)")
    return meta

df = load_cases()
PARAMS  = [c for c in df.columns if c.startswith("p_")]
PARAM_META = load_param_meta(df, [c for c in df.columns if c.startswith("p_")])
# expose nominals for the KDE-overlay annotation lines (was hardcoded to 2 params)
NOMINAL = {p: v[0] for p, v in PARAM_META.items() if v[0] is not None}
METRICS = ["ground_roll_m", "Vlof_mps", "t_liftoff_s", "max_alt_m", "max_TAS_mps",
           "max_abs_p_dps", "max_abs_q_dps", "max_abs_r_dps", "max_AoA_deg",
           "min_L_over_Wcosg", "max_throttle",
           # landing / touchdown metrics (present only for landing-profile campaigns)
           "td_speed_mps", "td_sink_mps", "td_pitch_deg", "approach_max_sink_mps",
           "flare_max_sink_mps", "flare_duration_s", "td_load_factor_est",
           "landing_roll_m", "landing_err_m"]
METRICS = [m for m in METRICS if m in df.columns]
LAND_METRICS = [m for m in ["td_speed_mps", "td_sink_mps", "td_pitch_deg",
                            "approach_max_sink_mps", "flare_max_sink_mps",
                            "flare_duration_s", "td_load_factor_est",
                            "landing_roll_m", "landing_err_m"] if m in df.columns]

# --------------------------------------------------------------------------- #
#  Wind condition (only present for wind campaigns, e.g. aws_monte_carlo_7)
# --------------------------------------------------------------------------- #
HAS_WIND   = "wind_case" in df.columns
WIND_ORDER = ["none", "head", "tail", "cross", "up", "down"]
WIND_COLORS = {"none": MUTED, "head": BLUE, "tail": "#f59e0b", "cross": "#a78bfa",
               "up": GREEN, "down": FAILURE_COLOR}
WIND_NUM   = [c for c in ["wind_speed_mps", "wind_dir_deg", "wind_dir_z_deg"]
              if c in df.columns]


def wind_types(d):
    """Wind types present in d, in canonical order (head/tail/... ) then any extras."""
    seen = set(d["wind_case"].unique())
    ordered = [w for w in WIND_ORDER if w in seen]
    return ordered + [w for w in sorted(seen) if w not in ordered]


# --------------------------------------------------------------------------- #
#  Failure criteria (present only when build was run with failure_criteria_config.json)
# --------------------------------------------------------------------------- #
HAS_CRITERIA = "fc_verdict" in df.columns
VERDICT_ORDER  = ["PASS", "PARTIAL", "FAIL"]
VERDICT_COLORS = {"PASS": SUCCESS_COLOR, "PARTIAL": PARTIAL_COLOR, "FAIL": FAILURE_COLOR}
# the per-criterion status columns are fc_<key>, excluding the _val/_run_s/summary fields
FC_SUMMARY = {"fc_verdict", "fc_n_failed", "fc_n_flagged", "fc_failed_list",
              "fc_flagged_list", "fc_phase_source"}
FC_KEYS = [c[3:] for c in df.columns
           if c.startswith("fc_") and c not in FC_SUMMARY
           and not c.endswith(("_val", "_run_s"))] if HAS_CRITERIA else []
STATUS_COLORS = {"ok": SUCCESS_COLOR, "flag": PARTIAL_COLOR, "fail": FAILURE_COLOR, "na": MUTED}


@st.cache_data
def load_criteria_config():
    """The human-edited thresholds file, for drawing limit lines / labels. {} if absent."""
    for name in ("failure_criteria_config.json", "../failure_criteria_config.json"):
        p = Path(name)
        if p.exists():
            try:
                return json.loads(p.read_text()).get("criteria", {})
            except Exception:
                return {}
    return {}


@st.cache_data
def load_oscillation():
    """Per-(case,channel,phase) oscillation long table, or None if not produced."""
    p = Path("oscillation.parquet")
    return pd.read_parquet(p) if p.exists() else None


def criterion_bounds(crit_cfg, key):
    """(lo, hi) for a criterion key, for threshold lines. None where unset."""
    c = crit_cfg.get(key, {})
    if c.get("max_abs") is not None:
        a = abs(float(c["max_abs"])); return -a, a
    lo = float(c["min"]) if c.get("min") is not None else None
    hi = float(c["max"]) if c.get("max") is not None else None
    return lo, hi


CRIT_CFG_META = load_criteria_config() if HAS_CRITERIA else {}


# --------------------------------------------------------------------------- #
#  Total-variation sensitivity analysis (the headline)
# --------------------------------------------------------------------------- #
def _density(samples, grid, dx):
    """Fast binned Gaussian KDE: histogram onto `grid`, smooth with a per-group
    Scott-bandwidth Gaussian. ~50x faster than scipy.gaussian_kde per call and,
    because the bandwidth is set from the group's own (n, std), it matches a
    direct gaussian_kde to <0.01 TV — but stays cheap enough for the permutation
    null over all 112 params."""
    n = len(samples)
    sd = np.std(samples)
    if n < 2 or sd == 0:
        return None
    sigma_pts = max(sd * n ** (-0.2) / dx, 0.6)        # Scott's rule, in grid points
    counts, _ = np.histogram(samples, bins=grid.size,
                             range=(grid[0] - dx / 2, grid[-1] + dx / 2))
    dens = gaussian_filter1d(counts.astype(float), sigma_pts, mode="constant")
    tot = dens.sum() * dx
    return dens / tot if tot > 0 else None


def _tv_distance(succ, fail, grid, dx):
    """TV = 0.5 * integral |f_fail - f_succ| dx between the two binned KDEs."""
    succ = succ[np.isfinite(succ)]
    fail = fail[np.isfinite(fail)]
    fs = _density(succ, grid, dx)
    ff = _density(fail, grid, dx)
    if fs is None or ff is None:
        return np.nan
    return float(0.5 * np.abs(ff - fs).sum() * dx)


@st.cache_data
def tv_ranking(_df, params, success_col="took_off", n_perm=200, grid_n=200, seed=0):
    """TV distance positive-vs-negative for every param + a permutation noise floor.

    `success_col` is the boolean column that defines the positive class (e.g.
    "landed" or "took_off"). Returns (ranking_df, floor_95, floor_99, n_neg). The
    floor is the *per-param* null: shuffle the class labels n_perm times, pool every
    param's TV under the shuffle, and take the 95th/99th percentile. A param above
    floor_95 is distinguishable from chance for this number of negatives. (Floor
    scales ~1/sqrt of n_neg, so a broader negative class sharpens secondary rankings.)
    """
    succ_mask = _df[success_col].values
    rng = np.random.default_rng(seed)

    grids, obs = {}, {}
    for p in params:
        x = _df[p].values.astype(float)
        finite = x[np.isfinite(x)]
        if len(finite) < 2 or finite.min() == finite.max():
            continue
        lo, hi = finite.min(), finite.max()
        pad = 0.05 * (hi - lo)
        g = np.linspace(lo - pad, hi + pad, grid_n)
        grids[p] = (g, g[1] - g[0])
        obs[p] = _tv_distance(x[succ_mask], x[~succ_mask], *grids[p])

    ranking = (pd.DataFrame({"param": list(obs.keys()), "tv": list(obs.values())})
               .dropna().sort_values("tv", ascending=False).reset_index(drop=True))

    n_fail = int((~succ_mask).sum())
    vals = {p: _df[p].values.astype(float) for p in grids}
    pooled = []
    for _ in range(n_perm):
        perm = rng.permutation(succ_mask)
        for p, (g, dx) in grids.items():
            tv = _tv_distance(vals[p][perm], vals[p][~perm], g, dx)
            if np.isfinite(tv):
                pooled.append(tv)
    pooled = np.asarray(pooled)
    floor_95 = float(np.percentile(pooled, 95)) if pooled.size else np.nan
    floor_99 = float(np.percentile(pooled, 99)) if pooled.size else np.nan
    return ranking, floor_95, floor_99, n_fail


def kde_overlay(df_all, param, success_col="took_off",
                pos_label="Success", neg_label="Failure", grid_n=200):
    """Overlaid positive/negative KDE figure for one parameter."""
    x = df_all[param].values.astype(float)
    succ_mask = df_all[success_col].values
    finite = x[np.isfinite(x)]
    lo, hi = finite.min(), finite.max()
    pad = 0.05 * (hi - lo) if hi > lo else 1.0
    grid = np.linspace(lo - pad, hi + pad, grid_n)
    fig = go.Figure()
    for mask, name, color in [(succ_mask, pos_label, SUCCESS_COLOR),
                              (~succ_mask, neg_label, FAILURE_COLOR)]:
        d = x[mask & np.isfinite(x)]
        if len(d) >= 2 and np.std(d) > 0:
            dens = gaussian_kde(d)(grid)
            fig.add_trace(go.Scatter(x=grid, y=dens, name=f"{name} (n={len(d)})",
                                     fill="tozeroy", mode="lines",
                                     line=dict(color=color)))
        # rug marks for the (few) negative cases
        if name == neg_label and len(d):
            fig.add_trace(go.Scatter(x=d, y=np.zeros_like(d), mode="markers",
                                     name=f"{neg_label} cases", showlegend=False,
                                     marker=dict(color=color, symbol="line-ns-open",
                                                 size=10)))
    if param in NOMINAL:
        fig.add_vline(x=NOMINAL[param], line_dash="dot", line_color="gray",
                      annotation_text=f"nominal {NOMINAL[param]:g}")
    fig.update_layout(title=f"{param}: success vs failure (KDE)",
                      xaxis_title=param, yaxis_title="probability density",
                      height=420)
    return fig


# light, low-opacity phase-shading colours for the drill-down panels
PHASE_COLORS = {"GROUND": "#7c8aa5", "TAKEOFF": "#7c8aa5", "CLIMB": BLUE, "CRUISE": GREEN,
                "LOITER": "#22d3ee", "APPROACH": "#f59e0b", "FLARE": FAILURE_COLOR,
                "ROLLOUT": "#a78bfa"}


def _chan(cts, name):
    """Return a display channel from the per-case timeseries, deriving it from raw columns when
    the precomputed one is absent (keeps the figure working on older timeseries.parquet files)."""
    if name in cts:
        return cts[name]
    raw = {"roll_deg": "phi", "pitch_deg": "theta", "yaw_deg": "psi",
           "p_dps": "p", "q_dps": "q", "r_dps": "r",
           "elevator_deg": "delta_e", "rudder_deg": "delta_r"}
    cols = set(cts.columns)
    if name in raw and raw[name] in cols:
        return np.degrees(cts[raw[name]])
    if name == "aileron_deg":
        for c in ("delta_a", "delta_aL"):
            if c in cols:
                return np.degrees(cts[c])
    if name == "climb_mps" and "V_ned_gnd_2" in cols:
        return -cts["V_ned_gnd_2"]
    if name == "ground_speed_mps" and {"V_ned_gnd_0", "V_ned_gnd_1"} <= cols:
        return np.hypot(cts["V_ned_gnd_0"], cts["V_ned_gnd_1"])
    if name == "beta_deg" and {"V_b_tas_0", "V_b_tas_1"} <= cols:
        return np.degrees(np.arctan2(cts["V_b_tas_1"], cts["V_b_tas_0"]))
    if name == "throttle_pct" and "mot0_thr_cmd" in cols:
        return cts["mot0_thr_cmd"] * 100.0
    return None


def telemetry_figure(cts):
    """Six-panel mission telemetry (states + controls) with flight-phase shading."""
    T = cts["Time_s"]
    rows = [
        ("Altitude & speed",               True),    # alt [m] | speeds [m/s]
        ("Euler angles [deg]",             False),   # roll / pitch / yaw
        ("Body rates [deg/s]",             False),   # p / q / r
        ("Flight-path γ, climb, throttle", True),    # γ [deg], climb [m/s] | throttle [%]
        ("AoA α & sideslip β [deg]",       False),
        ("Control deflections [deg]",      False),   # aileron / elevator / rudder
    ]
    fig = make_subplots(rows=len(rows), cols=1, shared_xaxes=True, vertical_spacing=0.028,
                        row_heights=[1.25, 1, 1, 1, 1, 1],
                        subplot_titles=[t for t, _ in rows],
                        specs=[[{"secondary_y": sec}] for _, sec in rows])

    def line(row, name, label, color, dash=None, sec=False):
        y = _chan(cts, name)
        if y is not None:
            fig.add_trace(go.Scatter(x=T, y=y, name=label, mode="lines",
                                     line=dict(color=color, width=1.6, dash=dash)),
                          row=row, col=1, secondary_y=sec)

    # 1 — altitude + airspeed + ground speed
    line(1, "alt_agl_m", "alt AGL [m]", BLUE)
    line(1, "TAS_mps", "airspeed [m/s]", CRIMSON, sec=True)
    line(1, "ground_speed_mps", "ground speed [m/s]", "#f59e0b", dash="dash", sec=True)
    fig.add_hline(y=TAKEOFF_ALT_M, line_dash="dot", line_color=MUTED, line_width=1, row=1, col=1)
    # 2 — Euler angles
    line(2, "roll_deg", "roll", BLUE); line(2, "pitch_deg", "pitch", "#f59e0b"); line(2, "yaw_deg", "yaw", GREEN)
    # 3 — body rates
    line(3, "p_dps", "p (roll rate)", BLUE); line(3, "q_dps", "q (pitch rate)", "#f59e0b"); line(3, "r_dps", "r (yaw rate)", GREEN)
    # 4 — flight-path angle / climb / throttle
    line(4, "gamma_deg", "γ flight-path [deg]", "#a78bfa"); line(4, "climb_mps", "climb [m/s]", GREEN, dash="dash")
    line(4, "throttle_pct", "throttle [%]", "#8aa0c8", sec=True)
    # 5 — AoA & sideslip
    line(5, "alpha_deg", "α (AoA)", CRIMSON); line(5, "beta_deg", "β (sideslip)", BLUE)
    # 6 — control deflections
    line(6, "aileron_deg", "aileron", BLUE); line(6, "elevator_deg", "elevator", GREEN); line(6, "rudder_deg", "rudder", "#f59e0b")

    fig.update_yaxes(title_text="alt [m]", row=1, col=1)
    fig.update_yaxes(title_text="speed [m/s]", row=1, col=1, secondary_y=True)
    fig.update_yaxes(title_text="throttle [%]", range=[0, 103], showgrid=False,
                     row=4, col=1, secondary_y=True)
    fig.update_xaxes(title_text="Time [s]", row=len(rows), col=1)
    fig.update_annotations(font=dict(size=12))   # style the subplot titles BEFORE adding phase labels
    fig.update_layout(height=1500, hovermode="x unified", margin=dict(t=60, b=10),
                      legend=dict(orientation="h", y=1.04, x=0, font=dict(size=10)))

    # phase shading across all panels + a phase label per band at the top
    if "phase" in cts and len(cts):
        ph = cts["phase"].astype(str).values
        t = np.asarray(T, dtype=float)
        edges = np.flatnonzero(ph[1:] != ph[:-1]) + 1
        starts, ends = np.r_[0, edges], np.r_[edges, len(ph)]
        for s, e in zip(starts, ends):
            x0, x1, name = float(t[s]), float(t[min(e, len(t) - 1)]), ph[s]
            fig.add_vrect(x0=x0, x1=x1, fillcolor=PHASE_COLORS.get(name, "#444444"),
                          opacity=0.10, line_width=0, layer="below", row="all", col=1)
            fig.add_annotation(x=0.5 * (x0 + x1), y=1.0, xref="x", yref="paper", yshift=8,
                               text=name, showarrow=False, font=dict(size=9, color=MUTED))
    return fig


# --------------------------------------------------------------------------- #
#  Sidebar — global filters
# --------------------------------------------------------------------------- #
sidebar_brand()
st.sidebar.header("Filters")
outcome_sel = st.sidebar.radio(
    "Outcome", ["All", OUTCOME_SUCCESS, OUTCOME_PARTIAL, OUTCOME_FAIL])
mask = pd.Series(True, index=df.index)
if outcome_sel != "All":
    mask &= df["outcome"].eq(outcome_sel)

st.sidebar.markdown("**Parameter range**")
slider_param = st.sidebar.selectbox("Restrict on parameter", ["(none)"] + PARAMS)
if slider_param != "(none)":
    lo, hi = float(df[slider_param].min()), float(df[slider_param].max())
    rng = st.sidebar.slider(slider_param, lo, hi, (lo, hi))
    mask &= df[slider_param].between(*rng)

if HAS_WIND:
    st.sidebar.markdown("**Wind**")
    wind_on_sel = st.sidebar.radio("Condition", ["All", "Wind ON", "Wind OFF"],
                                   horizontal=True)
    if wind_on_sel == "Wind ON":
        mask &= df["wind_on"]
    elif wind_on_sel == "Wind OFF":
        mask &= ~df["wind_on"]

    all_types = wind_types(df)
    wsel = st.sidebar.multiselect("Wind type", all_types, default=all_types)
    if wsel and len(wsel) < len(all_types):
        mask &= df["wind_case"].isin(wsel)

    on_speeds = sorted({float(s) for s in df.loc[df["wind_on"], "wind_speed_mps"]
                        if np.isfinite(s)})
    if len(on_speeds) > 1:
        smin, smax = on_speeds[0], on_speeds[-1]
        srng = st.sidebar.slider("Wind speed [m/s]  (wind-on cases)",
                                 smin, smax, (smin, smax))
        # constrain wind-on cases to the speed band; wind-off (0 m/s) cases pass through
        mask &= df["wind_speed_mps"].between(*srng) | ~df["wind_on"]

if HAS_CRITERIA:
    st.sidebar.markdown("**Failure criteria**")
    vbasis = st.sidebar.radio(
        "Verdict basis", ["Final (mission + criteria)", "Criteria only", "Mission only"],
        help="Final = worse of the mission outcome and the threshold-based criteria verdict.")
    vcol = {"Final (mission + criteria)": "final_verdict",
            "Criteria only": "criteria_verdict"}.get(vbasis)
    if vcol is not None:
        vsel = st.sidebar.multiselect("Verdict", VERDICT_ORDER, default=VERDICT_ORDER)
        if vsel and len(vsel) < len(VERDICT_ORDER):
            mask &= df[vcol].isin(vsel)

fdf = df[mask]
st.sidebar.caption(f"{len(fdf)} / {len(df)} cases selected")
st.sidebar.caption(
    f"Outcome (derived from data):\n"
    f"- **{OUTCOME_SUCCESS}** — touchdown reached (`land_result == landed`)\n"
    f"- **{OUTCOME_PARTIAL}** — peak AGL ≥ {TAKEOFF_ALT_M:g} m but never landed\n"
    f"- **{OUTCOME_FAIL}** — never reached {TAKEOFF_ALT_M:g} m AGL")

st.markdown('<div class="lat-eyebrow-main">LAT Aerospace</div>', unsafe_allow_html=True)
st.title("Monte Carlo Simulation Results")

tab_names = ["Overview", "Sensitivity", "Failure map", "Case drill-down"]
if HAS_WIND:
    tab_names.insert(1, "Wind")                       # right after Overview
if HAS_CRITERIA:
    tab_names.insert(1, "Statistics")                 # pushed right by the next insert
    tab_names.insert(1, "Failure criteria")           # first detail tab after Overview
_tabs = dict(zip(tab_names, st.tabs(tab_names)))
tab_overview    = _tabs["Overview"]
tab_sensitivity = _tabs["Sensitivity"]
tab_map         = _tabs["Failure map"]
tab_case        = _tabs["Case drill-down"]
tab_wind        = _tabs.get("Wind")
tab_crit        = _tabs.get("Failure criteria")
tab_stats       = _tabs.get("Statistics")

# --------------------------------------------------------------------------- #
with tab_overview:
    st.subheader("Campaign summary")
    n      = len(df)
    n_succ = int(df["landed"].sum())
    n_part = int((df["took_off"] & ~df["landed"]).sum())
    n_fail = int((~df["took_off"]).sum())
    pct = (lambda k: f"{100*k/n:.1f}%" if n else "—")
    c1, c2, c3, c4 = st.columns(4)
    c1.metric("Cases", n)
    c2.metric("Landed — full success", f"{n_succ}",
              help=f"{pct(n_succ)} — touchdown reached (land_result == 'landed').")
    c3.metric("Took off only — partial", f"{n_part}",
              help=f"{pct(n_part)} — reached ≥{TAKEOFF_ALT_M:g} m AGL but never landed.")
    c4.metric("Failed takeoff", f"{n_fail}",
              help=f"{pct(n_fail)} — never reached {TAKEOFF_ALT_M:g} m AGL.")
    crashes = int((df["max_alt_m"] < 5).sum()) if "max_alt_m" in df else 0
    st.caption(f"Full-success (landing) rate **{pct(n_succ)}** · "
               f"took-off-or-better **{pct(n_succ + n_part)}** · "
               f"crashes (max_alt < 5 m) **{crashes}**")

    st.plotly_chart(
        px.histogram(df, x="exit_reason", color="outcome",
                     color_discrete_map=OUTCOME_COLORS,
                     category_orders={"outcome": OUTCOME_ORDER},
                     title="Exit reasons by outcome"),
        use_container_width=True)

    st.markdown("##### Performance distributions  *(filtered selection)*")
    dist_cols = [c for c in ["ground_roll_m", "Vlof_mps", "max_alt_m", "max_TAS_mps",
                             "max_AoA_deg", "min_L_over_Wcosg"] if c in fdf.columns]

    BIN_SIZE = {"ground_roll_m":10, "Vlof_mps":1, "max_TAS_mps":1, "max_AoA_deg": 2}
    grid = st.columns(3)
    for i, col in enumerate(dist_cols):
        fig = px.histogram(fdf, x=col, title=col)
        if col in BIN_SIZE:
            fig.update_traces(xbins=dict(start=0, size = BIN_SIZE[col]))
        else:
            fig.update_traces(nbinsx=40)

        fig.update_layout(height=280, margin=dict(t=40, b=20, l=10, r=10),
                          showlegend=False)
        grid[i % 3].plotly_chart(fig, use_container_width=True)

    # ---- Landing performance (touchdown metrics; landed cases only) ----
    land_df = fdf[fdf["landed"]] if "landed" in fdf.columns else fdf.iloc[0:0]
    if LAND_METRICS and len(land_df):
        st.markdown(f"##### Landing performance  *(landed cases in selection — n={len(land_df)})*")
        LAND_BIN = {"td_speed_mps": 0.5, "td_sink_mps": 0.25, "td_pitch_deg": 1.0,
                    "approach_max_sink_mps": 0.5, "flare_max_sink_mps": 0.25,
                    "flare_duration_s": 0.5, "td_load_factor_est": 0.1,
                    "landing_roll_m": 5.0}      # landing_err_m left to auto (wide range)
        grid2 = st.columns(3)
        for i, col in enumerate(LAND_METRICS):
            fig = px.histogram(land_df, x=col, title=col)
            if col in LAND_BIN:
                xb = dict(size=LAND_BIN[col])
                if col != "td_pitch_deg":       # pitch can be negative -> don't pin start to 0
                    xb["start"] = 0
                fig.update_traces(xbins=xb)
            else:
                fig.update_traces(nbinsx=40)
            fig.update_layout(height=280, margin=dict(t=40, b=20, l=10, r=10),
                              showlegend=False)
            grid2[i % 3].plotly_chart(fig, use_container_width=True)
        st.caption("Touchdown speed/pitch/sink are measured at the flare→touchdown transition. "
                   "`td_load_factor_est` is a rough kinematic estimate (nz = 1 + a_up/g, 10 Hz "
                   "log, no modeled impact spike) — read it as a trend, not a certified load.")

# --------------------------------------------------------------------------- #
if HAS_WIND and tab_wind is not None:
  with tab_wind:
    st.subheader("Wind condition analysis")
    types = wind_types(df)
    on, off = df[df["wind_on"]], df[~df["wind_on"]]

    def _pct(d, col):
        return 100 * d[col].mean() if len(d) else float("nan")

    # ---- KPIs: wind ON vs OFF ----
    k1, k2, k3, k4 = st.columns(4)
    k1.metric("Wind-ON cases", f"{len(on)}",
              help="A named wind type with non-zero speed.")
    k2.metric("Wind-OFF cases", f"{len(off)}", help="wind_case = none (0 m/s).")
    to_on, to_off = _pct(on, "took_off"), _pct(off, "took_off")
    ld_on, ld_off = _pct(on, "landed"),   _pct(off, "landed")
    k3.metric("Takeoff rate — wind ON",
              "—" if np.isnan(to_on) else f"{to_on:.1f}%",
              delta=None if (np.isnan(to_on) or np.isnan(to_off)) else f"{to_on-to_off:+.1f} pts vs OFF",
              delta_color="normal")
    k4.metric("Landing rate — wind ON",
              "—" if np.isnan(ld_on) else f"{ld_on:.1f}%",
              delta=None if (np.isnan(ld_on) or np.isnan(ld_off)) else f"{ld_on-ld_off:+.1f} pts vs OFF",
              delta_color="normal")
    st.caption("Wind ON/OFF deltas are percentage-point differences vs the no-wind baseline. "
               "Cross-wind left/right share the type `cross`; their compass headings differ "
               "(see `wind_dir_deg`). Vertical gusts use `wind_dir_z_deg` (+90 up / −90 down).")

    # ---- Outcome composition by wind type (100% stacked) ----
    st.markdown("##### Outcome composition by wind type")
    counts = pd.crosstab(df["wind_case"], df["outcome"]).reindex(types).fillna(0)
    share  = counts.div(counts.sum(axis=1).replace(0, np.nan), axis=0) * 100
    n_by_type = counts.sum(axis=1).astype(int)
    figw = go.Figure()
    for oc in OUTCOME_ORDER:
        if oc in share.columns:
            figw.add_bar(x=[f"{t}<br>(n={n_by_type[t]})" for t in share.index],
                         y=share[oc], name=oc, marker_color=OUTCOME_COLORS[oc],
                         customdata=counts[oc].values,
                         hovertemplate=f"{oc}: %{{y:.1f}}%% (%{{customdata}} cases)<extra></extra>")
    figw.update_layout(barmode="stack", height=380, yaxis_title="share of cases [%]",
                       yaxis_range=[0, 100], margin=dict(t=30, b=10),
                       legend=dict(orientation="h", y=1.12, x=0))
    st.plotly_chart(figw, use_container_width=True)

    # ---- Which wind condition hurts most (ranked drop vs no-wind baseline) ----
    st.markdown("##### Which wind condition hurts most")
    hurt_kind = st.radio("Rank by drop in", ["Landing rate", "Takeoff rate"],
                         horizontal=True, key="wind_hurt_kind")
    hcol = "landed" if hurt_kind == "Landing rate" else "took_off"
    on_types = [t for t in types if t != "none"]
    base_rate = _pct(off, hcol)                       # no-wind baseline (NaN if no wind-off cases)
    hrows = []
    for t in on_types:
        sub = on[on["wind_case"] == t]
        if not len(sub):
            continue
        rate = _pct(sub, hcol)
        hrows.append(dict(wind=t, rate=rate, n=len(sub),
                          drop=(base_rate - rate) if not np.isnan(base_rate) else np.nan))
    if hrows:
        hdf = pd.DataFrame(hrows)
        has_base = not np.isnan(base_rate)
        # worst-first: biggest drop (or lowest rate when there's no baseline) at the TOP
        sort_col = "drop" if has_base else "rate"
        hdf = hdf.sort_values(sort_col, ascending=(not has_base))   # bar reads top=worst
        xvals = hdf["drop"] if has_base else hdf["rate"]
        figh = go.Figure(go.Bar(
            x=xvals, y=hdf["wind"], orientation="h",
            marker_color=[WIND_COLORS.get(w, BLUE) for w in hdf["wind"]],
            text=[(f"−{d:.1f} pts  ({r:.0f}% vs base {base_rate:.0f}%, n={n})" if has_base
                   else f"{r:.0f}%  (n={n})")
                  for d, r, n in zip(hdf["drop"], hdf["rate"], hdf["n"])],
            textposition="outside",
            hovertemplate="%{y}: %{x:.1f}<extra></extra>"))
        figh.update_layout(
            height=max(260, 40 * len(hdf)), margin=dict(l=10, r=120, t=30, b=10),
            xaxis_title=(f"{hurt_kind} drop vs no-wind baseline [pts]" if has_base
                         else f"{hurt_kind} [%]  (no no-wind baseline in this campaign)"))
        st.plotly_chart(figh, use_container_width=True)
        st.caption("Each wind type's "
                   + (f"**drop in {hurt_kind.lower()}** relative to the no-wind baseline "
                      f"({base_rate:.0f}%), sorted worst-first — the wind analogue of the "
                      "parameter TV-distance ranking."
                      if has_base else
                      f"absolute {hurt_kind.lower()} (no `none` cases exist in this campaign for a "
                      "baseline), sorted worst-first."))
    else:
        st.caption("No wind-on cases to rank.")

    # ---- Success rate vs wind speed, one line per type ----
    if len(on):
        st.markdown("##### Success rate vs wind speed")
        rate_kind = st.radio("Rate", ["Takeoff", "Landing"], horizontal=True,
                             key="wind_rate_kind")
        rcol = "took_off" if rate_kind == "Takeoff" else "landed"
        g = (on.groupby(["wind_case", "wind_speed_mps"])[rcol]
               .agg(rate="mean", n="size").reset_index())
        g["rate"] *= 100
        figr = px.line(g, x="wind_speed_mps", y="rate", color="wind_case",
                       markers=True, category_orders={"wind_case": types},
                       color_discrete_map=WIND_COLORS,
                       labels={"wind_speed_mps": "wind speed [m/s]",
                               "rate": f"{rate_kind.lower()} rate [%]",
                               "wind_case": "wind type"})
        base = _pct(off, rcol)
        if not np.isnan(base):
            figr.add_hline(y=base, line_dash="dot", line_color=MUTED,
                           annotation_text=f"no-wind baseline {base:.0f}%")
        figr.update_layout(height=400, yaxis_range=[0, 105], margin=dict(t=30, b=10))
        st.plotly_chart(figr, use_container_width=True)

    # ---- Metric distribution by wind type ----
    if METRICS:
        st.markdown("##### Performance metric by wind type")
        default_m = "ground_roll_m" if "ground_roll_m" in METRICS else METRICS[0]
        m_sel = st.selectbox("Metric", METRICS, index=METRICS.index(default_m))
        figb = px.box(df, x="wind_case", y=m_sel, color="wind_case", points="outliers",
                      category_orders={"wind_case": types}, color_discrete_map=WIND_COLORS)
        figb.update_layout(height=420, showlegend=False, margin=dict(t=30, b=10),
                           xaxis_title="wind type", yaxis_title=m_sel)
        st.plotly_chart(figb, use_container_width=True)

    # ---- Summary table by (wind type, speed) ----
    st.markdown("##### Summary by wind type & speed")
    aggs = {"cases": ("case_id", "size"),
            "takeoff_%": ("took_off", lambda s: round(100 * s.mean(), 1)),
            "landing_%": ("landed",   lambda s: round(100 * s.mean(), 1))}
    if "ground_roll_m" in df.columns:
        aggs["mean_ground_roll_m"] = ("ground_roll_m", lambda s: round(s.mean(), 1))
    if "landing_err_m" in df.columns:
        aggs["mean_landing_err_m"] = ("landing_err_m", lambda s: round(s.mean(), 1))
    summary = (df.groupby(["wind_case", "wind_speed_mps"]).agg(**aggs).reset_index()
                 .sort_values(["wind_case", "wind_speed_mps"]))
    st.dataframe(summary, use_container_width=True, hide_index=True)

# --------------------------------------------------------------------------- #
if HAS_CRITERIA and tab_crit is not None:
  with tab_crit:
    st.subheader("Failure criteria")
    n = len(df)
    vc = df["final_verdict"].value_counts()
    cv = df["criteria_verdict"].value_counts()
    k1, k2, k3, k4 = st.columns(4)
    k1.metric("Cases graded", n)
    k2.metric("Final PASS", f"{int(vc.get('PASS', 0))}",
              help=f"{100*int(vc.get('PASS',0))/n:.1f}% — passes both mission outcome and all criteria.")
    k3.metric("Final PARTIAL", f"{int(vc.get('PARTIAL', 0))}",
              help="Partial success or a sustained breach of a 'degraded' criterion.")
    k4.metric("Final FAIL", f"{int(vc.get('FAIL', 0))}",
              help="No takeoff, or a sustained breach of a 'critical' criterion.")
    st.caption("**Final verdict** = worse of the mission outcome and the threshold criteria. "
               f"Criteria-only verdict: PASS {int(cv.get('PASS',0))} · PARTIAL {int(cv.get('PARTIAL',0))} · "
               f"FAIL {int(cv.get('FAIL',0))}. Critical sustained breach → FAIL; degraded sustained → PARTIAL; "
               "a transient excursion that recovers within its grace interval is flagged but never fails.")

    # ---- outcome segmentation (pooling all cases is hard to read) ----
    OUTCOME_OPTS = [OUTCOME_SUCCESS, OUTCOME_PARTIAL, OUTCOME_FAIL]
    seg = st.radio("Segment by mission outcome", ["Compare all"] + OUTCOME_OPTS,
                   horizontal=True, key="fc_segment",
                   help="Pooling every case together is hard to read — 'No takeoff' cases trip many "
                        "criteria simply because they never flew. Compare the groups side by side, "
                        "or focus the whole tab on one.")
    compare = (seg == "Compare all")
    sdf = fdf if compare else fdf[fdf["outcome"] == seg]
    groups = [g for g in OUTCOME_OPTS if (fdf["outcome"] == g).any()]
    st.caption(f"{len(sdf)} cases in view"
               + ("  ·  comparing outcome groups side by side" if compare
                  else f"  ·  mission outcome = **{seg}**"))

    # ---- breach rate per criterion ----
    if compare:
        st.markdown("##### Fail rate per criterion × outcome group")
        rows = []
        for key in FC_KEYS:
            rec, any_eval = {"criterion": key}, False
            for g in groups:
                s = fdf.loc[fdf["outcome"] == g, f"fc_{key}"].astype(str)
                nn = int((s != "na").sum())
                rec[g] = (100 * int((s == "fail").sum()) / nn) if nn else np.nan
                any_eval = any_eval or nn > 0
            if any_eval:
                rows.append(rec)
        if rows:
            hm = pd.DataFrame(rows).set_index("criterion")[groups]
            hm = hm.reindex(hm.max(axis=1).sort_values(ascending=False).index)   # worst on top
            figb = px.imshow(hm, color_continuous_scale="Reds", aspect="auto",
                             text_auto=".0f", origin="upper", labels=dict(color="fail %"))
            figb.update_layout(height=max(360, 28 * len(hm)), margin=dict(l=10, r=20, t=30),
                               xaxis_title="mission outcome group")
            st.plotly_chart(figb, use_container_width=True)
            st.caption("Each cell = % of that outcome group's cases that FAIL the criterion "
                       "(`na` excluded). Criteria that fail mostly under 'No takeoff' are symptoms of "
                       "never flying; criteria that fail even among 'Landed' flights are the ones that "
                       "actually bite good flights.")
        else:
            st.info("No criteria evaluable across these outcome groups.")
    else:
        st.markdown("##### Breach rate per criterion")
        brk = []
        for key in FC_KEYS:
            s = sdf[f"fc_{key}"].astype(str)
            nn = int((s != "na").sum())
            if nn == 0:
                continue
            brk.append(dict(criterion=key, n=nn,
                            flag=100 * int((s == "flag").sum()) / nn,
                            fail=100 * int((s == "fail").sum()) / nn))
        if brk:
            bdf = pd.DataFrame(brk).sort_values("fail")
            figb = go.Figure()
            figb.add_bar(y=bdf.criterion, x=bdf["flag"], name="flag (transient/within grace)",
                         orientation="h", marker_color=PARTIAL_COLOR)
            figb.add_bar(y=bdf.criterion, x=bdf["fail"], name="fail (sustained)",
                         orientation="h", marker_color=FAILURE_COLOR)
            figb.update_layout(barmode="stack", height=max(340, 26 * len(bdf)),
                               xaxis_title="% of evaluated cases (na excluded)",
                               margin=dict(l=10, r=20, t=30),
                               legend=dict(orientation="h", y=1.07, x=0))
            st.plotly_chart(figb, use_container_width=True)
        else:
            st.info("No criteria are evaluable in this outcome group.")

    # ---- criteria driving complete failures ----
    st.markdown("##### Criteria driving complete failures")
    fail_df = sdf[sdf["final_verdict"] == "FAIL"]
    cnt = Counter()
    for lst in fail_df.get("fc_failed_list", pd.Series([], dtype=str)).fillna("").astype(str):
        for k in lst.split("|"):
            if k:
                cnt[k] += 1
    if cnt:
        cdf = pd.DataFrame(sorted(cnt.items(), key=lambda z: z[1]), columns=["criterion", "cases"])
        figd = px.bar(cdf, x="cases", y="criterion", orientation="h",
                      color_discrete_sequence=[FAILURE_COLOR])
        figd.update_layout(height=max(300, 26 * len(cdf)), margin=dict(l=10, r=20, t=20),
                           xaxis_title=f"# of FAIL cases  (n_fail = {len(fail_df)})")
        st.plotly_chart(figd, use_container_width=True)
    else:
        st.caption("No FAIL cases in the current selection.")

    # ---- per-criterion value distribution with threshold lines ----
    st.markdown("##### Criterion value distribution")
    val_keys = [k for k in FC_KEYS if f"fc_{k}_val" in df.columns]
    sel = st.selectbox("Criterion", val_keys) if val_keys else None
    if sel:
        vcol, scol = f"fc_{sel}_val", f"fc_{sel}"
        sub = sdf[[vcol, scol]].copy()
        sub = sub[np.isfinite(pd.to_numeric(sub[vcol], errors="coerce"))]
        if len(sub):
            figh = px.histogram(sub, x=vcol, color=scol, nbins=40,
                                category_orders={scol: ["ok", "flag", "fail", "na"]},
                                color_discrete_map=STATUS_COLORS)
            lo, hi = criterion_bounds(CRIT_CFG_META, sel)
            for b, lab in [(lo, "min"), (hi, "max")]:
                if b is not None:
                    figh.add_vline(x=b, line_dash="dash", line_color=TEXT,
                                   annotation_text=lab)
            figh.update_layout(height=380, margin=dict(t=30, b=10),
                               xaxis_title=f"{sel} — worst observed value")
            st.plotly_chart(figh, use_container_width=True)
        else:
            st.caption("No finite values for this criterion in the selection.")

    # ---- control oscillation: command vs response, then per-phase matrix ----
    osc = load_oscillation()
    if osc is not None and len(osc):
        o = osc[osc["case_id"].isin(set(sdf["case_id"]))]
        has_role = "role" in o.columns
        ROLE_COLORS = {"command": BLUE, "actual": PARTIAL_COLOR, "rate": FAILURE_COLOR}
        ROLE_ORDER = ["command", "actual", "rate"]

        # --- command vs response (per axis) — the controller-vs-airframe diagnostic ---
        if has_role:
            st.markdown("##### Control oscillation — command vs response (per axis)")
            of = o[o["feasible"]] if "feasible" in o.columns else o
            BAR_LABELS = {"band_frac": "mean band-power fraction (> cutoff)  — how oscillatory",
                          "dom_hz": "mean dominant frequency [Hz]  — what frequency"}
            bar_metric = st.radio("Show", ["band_frac", "dom_hz"], horizontal=True, key="osc_bar_metric",
                                  help="band_frac = fraction of variance above the cutoff (how much oscillation); "
                                       "dom_hz = dominant frequency in Hz (compare a rate's frequency to a known "
                                       "airframe mode, e.g. short-period / Dutch roll).")
            if len(of):
                agg = of.groupby(["criterion", "role"])[bar_metric].mean().reset_index()
                figcr = px.bar(agg, x="criterion", y=bar_metric, color="role", barmode="group",
                               category_orders={"role": ROLE_ORDER}, color_discrete_map=ROLE_COLORS,
                               labels={bar_metric: BAR_LABELS[bar_metric], "criterion": "axis", "role": ""})
                figcr.update_layout(height=380, margin=dict(t=30, b=10),
                                    legend=dict(orientation="h", y=1.12, x=0))
                st.plotly_chart(figcr, use_container_width=True)
            st.caption("**Command** high ⇒ the controller is *commanding* oscillation → tune the controller "
                       "(gains/filter). **Actual / rate** high while command is low ⇒ airframe/actuator "
                       "self-oscillation → address structurally (damping / aero / actuator). The criterion "
                       "verdict is driven by the command channel; actual & rate are diagnostics. "
                       "Switch **Show → dom_hz** to read the frequency, and use **rms** in the matrix below for "
                       "amplitude — a high band-fraction only matters if the amplitude is real.")

        # --- per-phase matrix (all channels; optional role filter) ---
        st.markdown("##### Per phase")
        cc1, cc2 = st.columns([2, 3])
        mlabel = cc1.radio("Metric", ["band_frac", "dom_hz", "rms", "reversal_rate", "slew_sat"],
                           horizontal=True,
                           help="band_frac = fraction of variance above the cutoff (how oscillatory); "
                                "dom_hz = dominant frequency [Hz]; rms = amplitude (deg for surfaces, "
                                "deg/s for rates); reversal_rate = sign-changes/s (use on short phases); "
                                "slew_sat = fraction of time the demanded rate nears the slew limit (command only).")
        rsel = cc2.radio("Channels", ["all"] + ROLE_ORDER, horizontal=True) if has_role else "all"
        oo = o if rsel == "all" else o[o["role"] == rsel]
        if len(oo):
            piv = oo.pivot_table(index="channel", columns="phase", values=mlabel, aggfunc="mean")
            order = [p for p in ["GROUND", "CLIMB", "CRUISE", "LOITER", "APPROACH", "FLARE", "ROLLOUT"]
                     if p in piv.columns]
            piv = piv.reindex(columns=order + [c for c in piv.columns if c not in order])
            figm = px.imshow(piv, color_continuous_scale="Inferno", aspect="auto",
                             text_auto=".2f", origin="upper")
            figm.update_layout(height=max(260, 26 * len(piv)), margin=dict(t=30, b=10),
                               coloraxis_colorbar_title=mlabel)
            st.plotly_chart(figm, use_container_width=True)
        st.caption("⚠️ Sim logs at ~10 Hz → only oscillation **below ~5 Hz** (Nyquist) is observable; faster "
                   "limit-cycles are invisible at this log rate. `band_frac`/`dom_hz` need a spectrum, so they're "
                   "blank on short phases (FLARE/GROUND/CRUISE) — judge those by `reversal_rate`. `rms` is the "
                   "amplitude (deg for surfaces, deg/s for rates). Cell = mean over the selected cases.")

    # ---- fallback phase-source caveat ----
    if "fc_phase_source" in sdf.columns and (sdf["fc_phase_source"] == "fallback").any():
        nfb = int((sdf["fc_phase_source"] == "fallback").sum())
        st.warning(f"{nfb} selected case(s) had no flight.csv — phases were approximated from sim_output "
                   "(`fc_phase_source = 'fallback'`); their phase-scoped results are coarse.")

# --------------------------------------------------------------------------- #
if HAS_CRITERIA and tab_stats is not None:
  with tab_stats:
    st.subheader("Statistics — evaluated parameters")
    st.caption("Distribution of each parameter's per-case worst value over the current selection. "
               "For upper-limit parameters read p90/p95/p99; for lower-limit ones read p5/p50. "
               "fail% = sustained breach, flag% = transient (within grace).")
    skeys = [k for k in FC_KEYS if f"fc_{k}_val" in fdf.columns]
    srows = []
    for k in skeys:
        v = pd.to_numeric(fdf[f"fc_{k}_val"], errors="coerce")
        v = v[np.isfinite(v)]
        status = fdf[f"fc_{k}"].astype(str)
        nn = int((status != "na").sum())
        if len(v) == 0:
            continue
        lo, hi = criterion_bounds(CRIT_CFG_META, k)
        srows.append({
            "parameter": k, "n": nn,
            "fail_%": round(100 * (status == "fail").sum() / nn, 1) if nn else np.nan,
            "flag_%": round(100 * (status == "flag").sum() / nn, 1) if nn else np.nan,
            "mean": round(float(v.mean()), 2), "std": round(float(v.std()), 2),
            "min": round(float(v.min()), 2), "p5": round(float(v.quantile(.05)), 2),
            "p50": round(float(v.quantile(.50)), 2), "p90": round(float(v.quantile(.90)), 2),
            "p95": round(float(v.quantile(.95)), 2), "p99": round(float(v.quantile(.99)), 2),
            "max": round(float(v.max()), 2), "limit_min": lo, "limit_max": hi,
        })
    if not srows:
        st.info("No evaluated parameters with values in the current selection.")
    else:
        stats_df = pd.DataFrame(srows)
        st.dataframe(stats_df, use_container_width=True, hide_index=True)
        st.download_button("⬇ Download statistics (CSV)", stats_df.to_csv(index=False),
                           "criteria_statistics.csv", "text/csv")

        st.markdown("##### Distribution & percentiles")
        pk = st.selectbox("Parameter", skeys, key="stats_dist_param")
        vv = pd.to_numeric(fdf[f"fc_{pk}_val"], errors="coerce"); vv = vv[np.isfinite(vv)]
        if len(vv):
            figd = px.histogram(x=vv.values, nbins=50)
            figd.update_traces(marker_color=BLUE, showlegend=False)
            lo, hi = criterion_bounds(CRIT_CFG_META, pk)
            for b, lab in [(lo, "limit min"), (hi, "limit max")]:
                if b is not None:
                    figd.add_vline(x=b, line_dash="dash", line_color=FAILURE_COLOR, annotation_text=lab)
            for q, lab in [(.90, "p90"), (.95, "p95"), (.99, "p99")]:
                figd.add_vline(x=float(vv.quantile(q)), line_dash="dot", line_color=PARTIAL_COLOR,
                               annotation_text=lab)
            figd.update_layout(height=360, margin=dict(t=30, b=10), showlegend=False,
                               xaxis_title=f"{pk} — per-case worst value")
            st.plotly_chart(figd, use_container_width=True)

        if HAS_WIND:
            st.markdown("##### By wind condition")
            order = {w: i for i, w in enumerate(wind_types(fdf))}
            g = (fdf.assign(_v=pd.to_numeric(fdf[f"fc_{pk}_val"], errors="coerce"))
                    .groupby("wind_case")["_v"]
                    .agg(n="count", mean="mean", p50=lambda s: s.quantile(.5),
                         p90=lambda s: s.quantile(.9), p95=lambda s: s.quantile(.95), max="max")
                    .reset_index().round(2))
            g = g.sort_values("wind_case", key=lambda s: s.map(order)).reset_index(drop=True)
            st.dataframe(g, use_container_width=True, hide_index=True)

        st.markdown("##### What drives exceedance (FDM parameters)")
        dk = st.selectbox("Parameter ", skeys, key="stats_drv_param")
        ds = df[f"fc_{dk}"].astype(str)
        fail = (ds == "fail").values; okm = (ds == "ok").values
        if fail.sum() >= 5 and okm.sum() >= 5:
            eff = []
            for p in PARAMS:
                x = df[p].values.astype(float)
                a = x[fail]; a = a[np.isfinite(a)]
                b = x[okm];  b = b[np.isfinite(b)]
                if len(a) < 3 or len(b) < 3:
                    continue
                sp = float(np.sqrt((a.var() + b.var()) / 2.0))
                if sp <= 0:
                    continue
                eff.append((p, (a.mean() - b.mean()) / sp))
            if eff:
                e = pd.DataFrame(eff, columns=["param", "d"])
                top = e.reindex(e["d"].abs().sort_values(ascending=False).index).head(12).iloc[::-1]
                fige = go.Figure(go.Bar(x=top["d"], y=top["param"], orientation="h",
                                        marker_color=np.where(top["d"] > 0, FAILURE_COLOR, BLUE)))
                fige.update_layout(height=max(320, 26 * len(top)), margin=dict(l=10, r=20, t=20),
                                   xaxis_title=f"effect size on '{dk}' exceedance  (+ = higher param ⇒ more fails)")
                st.plotly_chart(fige, use_container_width=True)
                st.caption(f"Cohen's d between FAIL ({int(fail.sum())}) and OK ({int(okm.sum())}) cases per FDM "
                           "parameter (|d|≳0.5 moderate, ≳0.8 strong). Sign shows direction of the driver.")
        else:
            st.caption(f"'{dk}' has too few fail/ok cases to attribute FDM drivers.")
        if HAS_WIND and fail.sum():
            wr = (df.assign(_f=fail).groupby("wind_case")["_f"].mean().mul(100)
                    .reindex(wind_types(df)).dropna().round(0))
            st.caption("Exceedance (fail) rate by wind type — " +
                       " · ".join(f"{w} {int(r)}%" for w, r in wr.items()))

# --------------------------------------------------------------------------- #
with tab_sensitivity:
    st.subheader("Parameter sensitivity — total-variation ranking")
    contrast = st.radio("Rank parameters that separate…",
                        ["Landed vs not landed", "Took off vs no takeoff"],
                        horizontal=True,
                        help="Which binary outcome split to score each parameter against.")
    if contrast.startswith("Landed"):
        success_col, pos_label, neg_label = "landed", "Landed", "Not landed"
    else:
        success_col, pos_label, neg_label = "took_off", "Took off", "No takeoff"
    st.caption(f"TV = 0.5·∫|f_{neg_label} − f_{pos_label}| dx between the two outcome KDEs. "
               "The noise floor is a label-permutation null (per-param TV reachable by "
               f"chance for this many '{neg_label}' cases); params below it are "
               "indistinguishable from noise. Floor scales ~1/√(n).")

    n_perm = st.slider("Permutation iterations (noise floor)", 50, 500, 200, step=50)
    with st.spinner("Computing TV ranking + permutation noise floor…"):
        ranking, floor_95, floor_99, n_neg = tv_ranking(df, PARAMS, success_col, n_perm=n_perm)

    if ranking.empty:
        st.info(f"Nothing to rank — the current selection has only one class for "
                f"'{pos_label} vs {neg_label}'. Both classes must be present.")
    else:
        top_n = st.slider("Show top-N params", 5, min(60, len(ranking)), min(25, len(ranking)))
        top = ranking.head(top_n).iloc[::-1]   # reversed for horizontal bar
        above = top["tv"] >= floor_95
        fig = go.Figure(go.Bar(
            x=top["tv"], y=top["param"], orientation="h",
            marker_color=np.where(above, BLUE, "#3a4257"),
            marker_line_width=0,
            text=[f"{v:.2f}" for v in top["tv"]], textposition="outside",
            textfont=dict(size=11, color=MUTED),
            hovertemplate="%{y}: TV=%{x:.3f}<extra></extra>"))
        fig.add_vline(x=floor_95, line_dash="dash", line_color="#f59e0b",
                      annotation_text=f"95th-pct floor {floor_95:.2f}",
                      annotation_font_color="#f59e0b")
        fig.add_vline(x=floor_99, line_dash="dot", line_color=FAILURE_COLOR,
                      annotation_text=f"99th-pct {floor_99:.2f}",
                      annotation_font_color=FAILURE_COLOR)
        fig.update_layout(title=f"TV distance per parameter  ·  n_{neg_label.lower().replace(' ','_')} = {n_neg}",
                          xaxis_title="total-variation distance",
                          height=max(420, 24 * top_n),
                          margin=dict(l=10, r=40, t=50),
                          xaxis_range=[0, min(1.0, top["tv"].max() * 1.15)])
        st.plotly_chart(fig, use_container_width=True)

        sig = ranking[ranking["tv"] >= floor_95]
        st.caption(f"{len(sig)} params exceed the 95th-pct noise floor ({floor_95:.2f}): "
                   + ", ".join(f"{r.param} ({r.tv:.2f})" for r in sig.itertuples()))

    st.markdown(f"##### {pos_label} vs {neg_label} distribution for one parameter")
    p = st.selectbox("Parameter", list(ranking["param"]) + PARAMS,
                     index=0 if "p_prop_CT_1" in list(ranking["param"]) else 0)
    st.plotly_chart(kde_overlay(df, p, success_col, pos_label, neg_label),
                    use_container_width=True)

# --------------------------------------------------------------------------- #
with tab_map:
    st.subheader("2D failure map")
    axis_opts = PARAMS + METRICS + WIND_NUM
    c1, c2 = st.columns(2)
    cx = c1.selectbox("X", axis_opts,
                      index=axis_opts.index("p_prop_CT_1") if "p_prop_CT_1" in axis_opts else 0)
    cy = c2.selectbox("Y", axis_opts,
                      index=axis_opts.index("p_prop_CT_J") if "p_prop_CT_J" in axis_opts else 1)

    def _scatter(sub, name, color, size, opacity, symbol="circle", line_w=0):
        cd = np.column_stack([sub["case_id"],
                              sub.get("exit_reason", pd.Series("", index=sub.index)),
                              sub.get("max_alt_m", pd.Series(np.nan, index=sub.index))])
        return go.Scattergl(
            x=sub[cx], y=sub[cy], mode="markers", name=f"{name} ({len(sub)})",
            customdata=cd, marker=dict(color=color, size=size, opacity=opacity,
                                       symbol=symbol, line=dict(width=line_w, color="white")),
            hovertemplate=(f"{cx}=%{{x:.3g}}<br>{cy}=%{{y:.3g}}<br>"
                           "case %{customdata[0]} · %{customdata[1]} · "
                           "alt %{customdata[2]:.0f} m<extra></extra>"))

    landed_df  = fdf[fdf["landed"]]
    partial_df = fdf[fdf["took_off"] & ~fdf["landed"]]
    noto_df    = fdf[~fdf["took_off"]]
    fig = go.Figure()
    fig.add_trace(_scatter(landed_df,  OUTCOME_SUCCESS, SUCCESS_COLOR, 6, 0.40))         # base
    fig.add_trace(_scatter(partial_df, OUTCOME_PARTIAL, PARTIAL_COLOR, 10, 0.85, "diamond"))
    fig.add_trace(_scatter(noto_df,    OUTCOME_FAIL,    FAILURE_COLOR, 13, 0.95, "x", 1))  # top
    fig.update_layout(height=620, title=f"{cx} vs {cy}",
                      xaxis_title=cx, yaxis_title=cy,
                      legend=dict(itemsizing="constant"))
    # shade the CT_1 < 0.55 failure region on whichever axis it sits
    if cx == "p_prop_CT_1":
        fig.add_vrect(x0=fdf[cx].min(), x1=CT1_FAILURE_THRESHOLD, line_width=0,
                      fillcolor=FAILURE_COLOR, opacity=0.07)
        fig.add_vline(x=CT1_FAILURE_THRESHOLD, line_dash="dash", line_color=FAILURE_COLOR,
                      annotation_text=f"CT_1 ≈ {CT1_FAILURE_THRESHOLD}")
    if cy == "p_prop_CT_1":
        fig.add_hrect(y0=fdf[cy].min(), y1=CT1_FAILURE_THRESHOLD, line_width=0,
                      fillcolor=FAILURE_COLOR, opacity=0.07)
        fig.add_hline(y=CT1_FAILURE_THRESHOLD, line_dash="dash", line_color=FAILURE_COLOR,
                      annotation_text=f"CT_1 ≈ {CT1_FAILURE_THRESHOLD}")
    st.plotly_chart(fig, use_container_width=True)
    st.caption("Failure ≈ p_prop_CT_1 ≲ 0.55, compounded by heavier drawn p_mass "
               "(lift can't exceed weight → min_L_over_Wcosg ≈ 1 → can't climb).")

# --------------------------------------------------------------------------- #
with tab_case:
    st.subheader("Per-case drill-down")
    ts = load_ts()

    # ---- Filter the case pool to a user-defined flight-profile region ----
    with st.expander("🔎 Find cases by flight-profile conditions", expanded=False):
        st.caption("Pick one or more fields and set a range (numeric) or allowed values "
                   "(categorical). A case must satisfy **all** conditions — the region you define. "
                   "The dropdown below then lists only the matching cases.")
        cat_cols = [c for c in ["outcome", "final_verdict", "criteria_verdict",
                                "land_result", "exit_reason", "wind_case"] if c in df.columns]
        if HAS_CRITERIA:                              # let them target a specific criterion's status
            cat_cols += [f"fc_{k}" for k in FC_KEYS]
        num_cols, _seen = [], set()
        for c in (METRICS + ["max_alt_m", "duration_s", "wind_speed_mps",
                             "wind_dir_deg", "wind_dir_z_deg"] + PARAMS):
            if c in df.columns and c not in _seen:
                _seen.add(c); num_cols.append(c)

        chosen = st.multiselect("Add conditions on…", cat_cols + num_cols, key="dd_cond_fields")
        cmask = pd.Series(True, index=df.index)
        for f in chosen:
            if f in cat_cols:
                opts = sorted(df[f].dropna().astype(str).unique().tolist())
                pick = st.multiselect(f"{f}  ∈", opts, default=opts, key=f"dd_cond_{f}")
                if pick and len(pick) < len(opts):
                    cmask &= df[f].astype(str).isin(pick)
                elif not pick:
                    cmask &= False                    # nothing selected -> no cases
            else:
                col = pd.to_numeric(df[f], errors="coerce")
                lo, hi = float(np.nanmin(col)), float(np.nanmax(col))
                if not (np.isfinite(lo) and np.isfinite(hi)) or lo == hi:
                    st.caption(f"`{f}` has no usable numeric range — skipped.")
                    continue
                r = st.slider(f, lo, hi, (lo, hi), key=f"dd_cond_{f}")
                cmask &= col.between(*r)               # NaNs (e.g. landing metrics on non-landed) drop out
        match = df[cmask]
        st.caption(f"**{len(match)}** / {len(df)} cases match these conditions.")

    notland_ids = sorted(match.loc[~match["landed"], "case_id"].tolist())
    only_bad = st.checkbox(f"Only matching cases that didn't land ({len(notland_ids)})", value=False)
    case_pool = notland_ids if only_bad else sorted(match["case_id"].tolist())
    if not case_pool:
        st.warning("No cases match the current conditions — widen the filter above.")
        st.stop()
    cid = st.selectbox(f"Case  ({len(case_pool)} match)", case_pool)

    row = df[df["case_id"] == cid].iloc[0]
    badge = {OUTCOME_SUCCESS: "✅ Landed — full success",
             OUTCOME_PARTIAL: "🟡 Took off, no landing — partial",
             OUTCOME_FAIL:    "❌ Failed takeoff"}[row["outcome"]]
    st.markdown(f"**Case {cid}** — {badge} · land_result = `{row['land_result']}` · "
                f"exit_reason = `{row['exit_reason']}`")

    if HAS_WIND:
        wc = row.get("wind_case", "none")
        ws = float(row.get("wind_speed_mps", 0.0) or 0.0)
        if wc == "none" or ws <= 0:
            st.markdown("Wind: **none** (0 m/s)")
        else:
            wdir = row.get("wind_dir_deg", float("nan"))
            wdz  = row.get("wind_dir_z_deg", float("nan"))
            bits = [f"**{wc}**", f"{ws:.1f} m/s"]
            if pd.notna(wdir): bits.append(f"dir {float(wdir):.0f}°")
            if pd.notna(wdz) and float(wdz) != 0: bits.append(f"dirZ {float(wdz):+.0f}°")
            st.markdown("Wind: " + " · ".join(bits))

    if HAS_CRITERIA:
        fv = str(row.get("final_verdict", "—"))
        cvd = str(row.get("criteria_verdict", "—"))
        vico = {"PASS": "✅", "PARTIAL": "🟡", "FAIL": "❌"}
        st.markdown(f"Verdict: {vico.get(fv, '')} **{fv}** (final) · criteria **{cvd}** · "
                    f"mission **{row['outcome']}**")
        failed = [f for f in str(row.get("fc_failed_list", "") or "").split("|") if f]
        flagged = [f for f in str(row.get("fc_flagged_list", "") or "").split("|") if f]
        if failed:
            st.markdown("**Failed:** " + " ".join(f"`{f}`" for f in failed))
        if flagged:
            st.markdown("**Flagged (recovered within grace):** " + " ".join(f"`{f}`" for f in flagged))
        if not failed and not flagged:
            st.caption("All criteria OK (or not applicable) for this case.")
        with st.expander("Per-criterion detail (this case)"):
            recs = []
            for key in FC_KEYS:
                lo, hi = criterion_bounds(CRIT_CFG_META, key)
                recs.append(dict(criterion=key, status=str(row.get(f"fc_{key}")),
                                 value=row.get(f"fc_{key}_val"),
                                 min=np.nan if lo is None else lo,
                                 max=np.nan if hi is None else hi,
                                 longest_run_s=row.get(f"fc_{key}_run_s")))
            det = pd.DataFrame(recs)
            for c in ("value", "min", "max", "longest_run_s"):
                det[c] = pd.to_numeric(det[c], errors="coerce")
            st.dataframe(det, use_container_width=True, hide_index=True)
        _osc = load_oscillation()
        if _osc is not None:
            oc = _osc[_osc["case_id"] == cid]
            if len(oc):
                with st.expander("Control-oscillation detail (this case · channel × phase)"):
                    oc_cols = [c for c in ["channel", "role", "phase", "n_samples", "feasible", "rms",
                                           "band_frac", "dom_hz", "reversal_rate", "slew_sat", "status"]
                               if c in oc.columns]
                    oc_sorted = oc.sort_values([c for c in ["criterion", "role", "phase"]
                                                if c in oc.columns])
                    st.dataframe(oc_sorted[oc_cols], use_container_width=True, hide_index=True)

    m1, m2, m3, m4 = st.columns(4)
    m1.metric("Ground roll", f"{row.get('ground_roll_m', float('nan')):.1f} m")
    m2.metric("V_lof", f"{row.get('Vlof_mps', float('nan')):.1f} m/s")
    m3.metric("Max alt", f"{row.get('max_alt_m', float('nan')):.0f} m")
    m4.metric("Max AoA", f"{row.get('max_AoA_deg', float('nan')):.1f}°")

    cts = ts[ts["case_id"] == cid].sort_values("Time_s")
    fig = telemetry_figure(cts)
    st.plotly_chart(fig, use_container_width=True)

    with st.expander("Outcomes + computed metrics"):
        st.dataframe(df[df["case_id"] == cid][
            ["outcome", "took_off", "landed", "mission_complete", "land_result",
             "max_alt_m", "duration_s"] + METRICS].T, use_container_width=True)
    with st.expander(f"Perturbed parameters ({len(PARAMS)}) — sorted by deviation from nominal"):
        nominal = pd.Series({p: PARAM_META[p][0] for p in PARAMS})
        sigma   = pd.Series({p: PARAM_META[p][1] for p in PARAMS})
        pv = pd.DataFrame({"value": df[df["case_id"] == cid][PARAMS].iloc[0]})
        pv["nominal"] = nominal
        pv["% err"]   = (pv["value"] - pv["nominal"]) \
                        / pv["nominal"].where(pv["nominal"].abs() > 1e-9) * 100
        pv["σ-dev"]   = (pv["value"] - pv["nominal"]) / sigma.where(sigma.abs() > 0)
        pv["description"] = pd.Series({p: PARAM_META[p][2] for p in PARAMS})
        # sort culprit-first: largest |deviation from nominal, in σ| at the top
        pv = pv.reindex(pv["σ-dev"].abs().sort_values(ascending=False,
                                                      na_position="last").index)

        culprit = pv.index[0]
        cz = pv.loc[culprit, "σ-dev"]
        st.caption(
            f"Most extreme draw: **{culprit}** at **{cz:+.2f}σ** "
            f"(value {pv.loc[culprit, 'value']:.4g} vs nominal {pv.loc[culprit, 'nominal']:.4g}) "
            "— likeliest contributor for this case. **σ-dev** = (value − nominal) / 1σ is the "
            "reliable culprit signal; **% err** is shown too but blows up for near-zero-nominal "
            "params (so the table is ranked and highlighted by σ-dev, not % err).")

        zmax = np.nanmax(np.abs(pv["σ-dev"].values))
        zmax = float(zmax) if np.isfinite(zmax) and zmax > 0 else 1.0

        def _row_style(r):
            z = r["σ-dev"]
            a = 0.0 if pd.isna(z) else min(abs(z) / zmax, 1.0) * 0.45   # redder = more extreme
            css = f"background-color: rgba(224,51,75,{a:.3f})"
            if r.name == culprit:
                css += "; font-weight:700"
            return [css] * len(r)

        sty = (pv.style
                 .format({"value": "{:.4g}", "nominal": "{:.4g}",
                          "% err": "{:+.1f}%", "σ-dev": "{:+.2f}σ"}, na_rep="—")
                 .apply(_row_style, axis=1))
        st.dataframe(sty, use_container_width=True, height=460)
