#!/usr/bin/env python3
"""
uSTOL single-run dashboard.

Pick (or upload) one FDM sim_output CSV and explore every signal interactively —
zoom/pan with the mouse, drag-select a time window, double-click to reset. This
replaces the "re-run the plot clipped to 120 s" workflow.

Run:
    streamlit run ustol_sims/dashboard/app.py
    ->  http://localhost:8501

CSV source: the dropdown lists ustol_sims/logs/sim_output_*.csv (newest first);
or upload any CSV. Desired-vs-actual overlays (pitch, FPA) are read from the
newest ArduPlane dataflash *.BIN under <workspace>/logs/ when present.
"""
from pathlib import Path
import math

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import streamlit as st

R2D = 180.0 / math.pi
HERE = Path(__file__).resolve().parent          # ustol_sims/dashboard/
USTOL = HERE.parent                             # ustol_sims/
LOGS = USTOL / "logs"
WS = USTOL.parent                               # workspace root
BIN_DIR = WS / "logs"
MASS_DEFAULT, G = 17.0, 9.81

st.set_page_config(page_title="uSTOL run dashboard", page_icon="✈", layout="wide")


# --------------------------------------------------------------------------- #
#  Data loading
# --------------------------------------------------------------------------- #
@st.cache_data(show_spinner=False)
def load_csv(path_or_bytes, name, mtime=None):
    df = pd.read_csv(path_or_bytes)
    # derived quantities
    if {"V_b_tas_0", "V_b_tas_2"}.issubset(df.columns):
        df["alpha"] = np.degrees(np.arctan2(df["V_b_tas_2"], df["V_b_tas_0"]))
        df.loc[np.hypot(df["V_b_tas_0"], df["V_b_tas_2"]) < 1.0, "alpha"] = 0.0
    if {"V_ned_gnd_0", "V_ned_gnd_1", "V_ned_gnd_2"}.issubset(df.columns):
        h = np.hypot(df["V_ned_gnd_0"], df["V_ned_gnd_1"])
        df["gamma"] = np.degrees(np.arctan2(-df["V_ned_gnd_2"], np.where(h > 0.5, h, 0.5)))
        df["climb"] = -df["V_ned_gnd_2"]
    for c in ("theta", "phi", "psi", "p", "q", "r", "p_dot", "q_dot", "r_dot",
              "delta_e", "delta_aL", "delta_aR", "delta_r", "delta_f"):
        if c in df.columns:
            df[c + "_deg"] = df[c] * R2D
    if "total_rotor_force" in df.columns:
        df["TW"] = df["total_rotor_force"] / (MASS_DEFAULT * G)
    return df


@st.cache_data(show_spinner=False)
def load_desired(bin_path, mtime=None):
    """Desired pitch (ATT.DesPitch) and FPA (asin(TECS.dhdem/spdem)) on raw sim time."""
    try:
        from pymavlink import mavutil
    except Exception:
        return None
    mb = mavutil.mavlink_connection(str(bin_path))
    at, te = [], []
    while True:
        m = mb.recv_match(type=["ATT", "TECS"], blocking=False)
        if m is None:
            break
        if m.get_type() == "ATT" and hasattr(m, "DesPitch"):
            at.append((m.TimeUS / 1e6, m.DesPitch))
        elif m.get_type() == "TECS":
            sp, dh = getattr(m, "spdem", None), getattr(m, "dhdem", None)
            if sp is not None and dh is not None:
                te.append((m.TimeUS / 1e6, sp, dh))
    out = {}
    if at:
        a = np.array(at); out["pitch"] = (a[:, 0], a[:, 1])
    if te:
        t = np.array(te); ratio = np.clip(np.where(t[:, 1] > 0.5, t[:, 2] / np.where(t[:, 1] > 0.5, t[:, 1], 1), 0), -1, 1)
        out["gamma"] = (t[:, 0], np.degrees(np.arcsin(ratio)))
    return out or None


def newest(globpat, folder):
    fs = sorted(folder.glob(globpat), key=lambda p: p.stat().st_mtime, reverse=True)
    return fs


# --------------------------------------------------------------------------- #
#  Sidebar — choose the run
# --------------------------------------------------------------------------- #
st.sidebar.title("✈ uSTOL run")
csvs = newest("sim_output_*.csv", LOGS)
options = [p.name for p in csvs]
up = st.sidebar.file_uploader("…or upload a CSV", type="csv")

df = name = None
if up is not None:
    name = up.name
    df = load_csv(up, name)
elif options:
    sel = st.sidebar.selectbox("CSV in logs/ (newest first)", options, index=0)
    p = LOGS / sel
    df = load_csv(p, sel, p.stat().st_mtime)
    name = sel
else:
    st.warning(f"No sim_output_*.csv in {LOGS}. Upload one in the sidebar.")
    st.stop()

t = df["Time_s"].to_numpy()
tmin, tmax = float(t.min()), float(t.max())
win = st.sidebar.slider("Time window (s)", tmin, tmax, (tmin, min(tmax, tmin + 120)),
                        step=max(0.5, round((tmax - tmin) / 500, 1)))
show_des = st.sidebar.checkbox("Overlay desired (from dataflash .BIN)", value=True)
mass = st.sidebar.number_input("Mass (kg) for T/W & L/Wcosγ", value=MASS_DEFAULT, step=0.5)

# desired overlay
des = None
if show_des:
    bins = newest("*.BIN", BIN_DIR)
    if bins:
        des = load_desired(bins[0], bins[0].stat().st_mtime)
        st.sidebar.caption(f"desired from {bins[0].name}")
    else:
        st.sidebar.caption("no .BIN found for desired overlay")

st.title(f"uSTOL telemetry — {name}")
st.caption(f"{len(df)} rows · t = {tmin:.1f}…{tmax:.1f} s · window {win[0]:.0f}…{win[1]:.0f} s "
           "· drag to zoom, double-click to reset")

m = (t >= win[0]) & (t <= win[1])
tw = t[m]


def col(c):
    return df[c].to_numpy()[m] if c in df.columns else None


def line(fig, y, label, row, c, dash=None, color=None):
    if y is None:
        return
    fig.add_trace(go.Scatter(x=tw, y=y, name=label, mode="lines",
                             line=dict(width=1.3, dash=dash, color=color)), row=row, col=c)


def desired_trace(fig, key, row, c, color):
    if des and key in des:
        dt, dy = des[key]
        mm = (dt >= win[0]) & (dt <= win[1])
        if mm.any():
            fig.add_trace(go.Scatter(x=dt[mm], y=dy[mm], name=f"{key} des", mode="lines",
                                     line=dict(width=1.6, dash="dash", color=color)), row=row, col=c)


# --------------------------------------------------------------------------- #
#  Tabs of grouped, interactive plots
# --------------------------------------------------------------------------- #
tab1, tab2, tab3, tab4 = st.tabs(["Trajectory & energy", "Attitude & AoA/FPA",
                                  "Rates & accels", "Aero & propulsion"])

with tab1:
    fig = make_subplots(rows=2, cols=2, shared_xaxes=True,
                        subplot_titles=("Altitude AGL (m)", "True airspeed (m/s)",
                                        "Climb rate (m/s)", "Energy: TAS vs thrust"))
    line(fig, col("alt_agl_m"), "alt", 1, 1)
    line(fig, col("TAS_mps"), "TAS", 1, 2)
    line(fig, col("climb"), "climb", 2, 1)
    line(fig, col("TAS_mps"), "TAS", 2, 2)
    if "total_rotor_force" in df.columns:
        fig.add_trace(go.Scatter(x=tw, y=col("total_rotor_force"), name="thrust (N)",
                                 yaxis="y", line=dict(width=1.0, color="firebrick")), row=2, col=2)
    fig.update_layout(height=620, margin=dict(t=40, b=10), legend=dict(orientation="h"))
    st.plotly_chart(fig, use_container_width=True)

with tab2:
    fig = make_subplots(rows=2, cols=2, shared_xaxes=True,
                        subplot_titles=("Euler angles (deg)", "AoA / FPA / pitch — solid=actual, dashed=desired",
                                        "Pitch θ: actual vs desired", "L / (W cos γ)"))
    line(fig, col("phi_deg"), "phi", 1, 1); line(fig, col("theta_deg"), "theta", 1, 1); line(fig, col("psi_deg"), "psi", 1, 1)
    line(fig, col("alpha"), "alpha", 1, 2, color="#1f77b4")
    line(fig, col("gamma"), "gamma", 1, 2, color="#ff7f0e")
    line(fig, col("theta_deg"), "theta", 1, 2, color="#2ca02c")
    desired_trace(fig, "gamma", 1, 2, "#ff7f0e")
    desired_trace(fig, "pitch", 1, 2, "#2ca02c")
    line(fig, col("theta_deg"), "theta act", 2, 1, color="#2ca02c")
    desired_trace(fig, "pitch", 2, 1, "#2ca02c")
    if {"Lift_N"}.issubset(df.columns) and "gamma" in df.columns:
        W = mass * G
        lw = col("Lift_N") / (W * np.cos(np.radians(col("gamma"))))
        line(fig, lw, "L/(Wcosγ)", 2, 2)
        fig.add_hline(y=1.0, line_dash="dash", line_color="black", row=2, col=2)
    fig.update_layout(height=620, margin=dict(t=40, b=10), legend=dict(orientation="h"))
    st.plotly_chart(fig, use_container_width=True)

with tab3:
    fig = make_subplots(rows=2, cols=2, shared_xaxes=True,
                        subplot_titles=("Body rates p/q/r (deg/s)", "Angular accel (deg/s²)",
                                        "Body accel (m/s²)", "Pitch rate q (deg/s)"))
    line(fig, col("p_deg"), "p", 1, 1); line(fig, col("q_deg"), "q", 1, 1); line(fig, col("r_deg"), "r", 1, 1)
    line(fig, col("p_dot_deg"), "p_dot", 1, 2); line(fig, col("q_dot_deg"), "q_dot", 1, 2); line(fig, col("r_dot_deg"), "r_dot", 1, 2)
    line(fig, col("Accel_b_0"), "ax", 2, 1); line(fig, col("Accel_b_1"), "ay", 2, 1); line(fig, col("Accel_b_2"), "az", 2, 1)
    line(fig, col("q_deg"), "q", 2, 2, color="#2ca02c")
    fig.update_layout(height=620, margin=dict(t=40, b=10), legend=dict(orientation="h"))
    st.plotly_chart(fig, use_container_width=True)

with tab4:
    fig = make_subplots(rows=2, cols=2, shared_xaxes=True,
                        subplot_titles=("Aero forces (N)", "Aero coefficients",
                                        "Control surfaces (deg)", "Thrust & T/W (all 18 EDFs)"))
    line(fig, col("Lift_N"), "Lift", 1, 1); line(fig, col("Drag_N"), "Drag", 1, 1); line(fig, col("Side_N"), "Side", 1, 1)
    line(fig, col("Lift_Coeff"), "CL", 1, 2); line(fig, col("Drag_Coeff"), "CD", 1, 2); line(fig, col("Moment_Coeff"), "Cm", 1, 2)
    for c, lbl in [("delta_e_deg", "elev"), ("delta_aL_deg", "ailL"), ("delta_aR_deg", "ailR"),
                   ("delta_r_deg", "rud"), ("delta_f_deg", "flap")]:
        line(fig, col(c), lbl, 2, 1)
    line(fig, col("total_rotor_force"), "thrust (N)", 2, 2, color="firebrick")
    if "TW" in df.columns:
        tw_series = col("total_rotor_force") / (mass * G)
        line(fig, tw_series, "T/W", 2, 2, color="#1f77b4")
    fig.update_layout(height=620, margin=dict(t=40, b=10), legend=dict(orientation="h"))
    st.plotly_chart(fig, use_container_width=True)

st.caption("Note: J / Cmu are not shown — they need the real per-EDF rpm, which "
           "isn't in the CSV (mot0_thr_cmd is a control motor, not the fleet throttle). "
           "Log s_motor[i].J and .Cmu from the FDM to add them.")
