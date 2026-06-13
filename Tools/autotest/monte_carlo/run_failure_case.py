#!/usr/bin/env python3
"""Re-run a single FAILURE case forwarding to Mission Planner, using the runner's
_run_case with that case's exact perturbation (from its campaign overrides.txt).
Writes to a separate folder so the campaign dataset is untouched."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import monte_carlo_runner as r

CASE     = 539
CAMPAIGN = os.path.abspath("mc_results_20260611_173053")
OUT      = os.path.abspath("mc_watch_failure")          # separate from campaign
os.makedirs(OUT, exist_ok=True)

config = r.load_config("monte_carlo_config_ustol_v1_watch.json")   # gcs_output -> MP

perturbed = {}
for line in open(os.path.join(CAMPAIGN, f"case_{CASE:04d}", "overrides.txt")):
    line = line.strip()
    if "=" in line and not line.startswith("#"):
        k, v = line.split("=", 1)
        perturbed[k] = float(v)
print(f"running FAILURE case {CASE} (prop_CT_1={perturbed.get('prop_CT_1'):.3f}), forwarding to MP")

res = r._run_case(CASE, 100, config, perturbed, OUT, str(r.WORKSPACE))
print("RESULT:", res)
