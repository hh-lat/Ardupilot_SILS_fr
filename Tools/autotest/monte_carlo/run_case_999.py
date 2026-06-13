#!/usr/bin/env python3
"""One-off: re-run the single Monte Carlo case 0999 (cut off when the campaign
was killed) using the runner's own _run_case, with case 999's exact perturbation
parsed from its already-written overrides.txt. Writes into the same results dir."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import monte_carlo_runner as r

RESULTS = os.path.abspath("mc_results_20260611_173053")
config  = r.load_config("monte_carlo_config_ustol_v1.json")

perturbed = {}
for line in open(os.path.join(RESULTS, "case_0999", "overrides.txt")):
    line = line.strip()
    if "=" in line and not line.startswith("#"):
        k, v = line.split("=", 1)
        perturbed[k] = float(v)
print(f"loaded {len(perturbed)} perturbed params for case 999")

res = r._run_case(999, 100, config, perturbed, RESULTS, str(r.WORKSPACE))
print("RESULT:", res)
