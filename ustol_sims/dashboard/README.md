# uSTOL single-run dashboard

Interactive viewer for one FDM `sim_output_*.csv`. Pick a run, explore every
signal, zoom/pan with the mouse — no more re-running clipped plots.

## Run
```bash
streamlit run ustol_sims/dashboard/app.py
# -> http://localhost:8501
```

## Use
- **Sidebar → CSV in logs/**: lists `ustol_sims/logs/sim_output_*.csv`, newest first.
  Or **upload** any CSV.
- **Time window** slider clips all plots; or just drag-select on any chart to zoom
  (double-click to reset).
- **Overlay desired**: reads the newest `*.BIN` under `<workspace>/logs/` and overlays
  `ATT.DesPitch` (desired pitch) and `asin(TECS.dhdem/spdem)` (desired FPA), dashed.

## Tabs
1. **Trajectory & energy** — altitude, TAS, climb, TAS-vs-thrust
2. **Attitude & AoA/FPA** — Euler, α/γ/θ with desired, θ actual-vs-desired, L/(W·cosγ)
3. **Rates & accels** — p/q/r, angular accel, body accel
4. **Aero & propulsion** — forces, coefficients, control surfaces, thrust & T/W

## Not shown: J / Cmu
The advance ratio and blown-wing Cmu need the real per-EDF rpm. `mot0_thr_cmd` is a
control (differential) motor, not the fleet throttle, so it can't be used to derive
them. To add J/Cmu, log `s_motor[i].J` and `s_motor[i].Cmu` from the FDM into the CSV.
