"""The ONLY module that imports the live campaign script aws_monte_carlo_11_edf.

The campaign runner is treated as a FROZEN LIBRARY: imported, never edited,
never copied, so the tuner gets identical episode semantics (same SITL boot
flags, watchdog, mission profile, metrics) to the campaign that will validate
the tuned gains. If the campaign script ever changes incompatibly, the
signature assert below fails loudly at import time — nothing else in the
package needs auditing.
"""
import inspect
import sys
from pathlib import Path

_MC_DIR = Path(__file__).resolve().parent.parent          # Tools/autotest/monte_carlo/
if str(_MC_DIR) not in sys.path:
    sys.path.insert(0, str(_MC_DIR))

import aws_monte_carlo_11_edf as mc                        # noqa: E402

# Frozen contract: the exact _run_case signature the tuner relies on.
_EXPECTED_RUN_CASE = (
    "case_id", "case_seed", "instance_id", "config", "perturbed", "output_dir",
    "workspace", "speedup", "save_plot", "hard_timeout", "params_file", "wind",
    "dt_enabled", "fail_case", "sim_parquet", "trim_cols", "keep_aux")
_actual = tuple(inspect.signature(mc._run_case).parameters)
if _actual != _EXPECTED_RUN_CASE:
    raise ImportError(
        "aws_monte_carlo_11_edf._run_case signature changed — the campaign "
        f"script drifted under the tuner.\n  expected: {_EXPECTED_RUN_CASE}\n"
        f"  actual:   {_actual}\nUpdate gain_tuner/mc_bridge.py deliberately.")

run_case              = mc._run_case
sample_parameters     = mc.sample_parameters
wind_params_for_case  = mc.wind_params_for_case
edf_mask_for          = mc.edf_mask_for
metrics_from_csv      = mc.metrics_from_csv
load_config           = mc.load_config
resolve_defaults_parm = mc.resolve_defaults_parm
kill_proc             = mc._kill_proc
FAILURE_SCENARIOS     = mc.FAILURE_SCENARIOS

SCRIPT_DIR  = mc.SCRIPT_DIR
WORKSPACE   = mc.WORKSPACE
BINARY      = mc.BINARY
DEFAULT_CFG = mc.DEFAULT_CFG
