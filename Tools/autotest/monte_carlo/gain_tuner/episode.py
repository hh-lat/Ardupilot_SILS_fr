"""One tuning episode = one SITL flight of the full campaign profile with a
candidate gain set under one CRN scenario, via the untouched campaign runner's
_run_case (identical boot flags, watchdog, mission, metrics).

One SITL boot per episode, no reuse: FDM physics overrides are startup-only
(LAT_MC_OVERRIDE_FILE), boot+EKF is <20% of episode wall time, and identical
boot semantics to the campaign means the tuned gains are evaluated exactly the
way the final validation campaign will fly them.
"""
import glob
import json
import os
import time
from dataclasses import dataclass, field

from . import episode_metrics, gain_space, mc_bridge, objective, scenarios


@dataclass
class TunerContext:
    base_cfg: dict
    base_parm: str
    workspace: str
    speedup: int = 10
    hard_timeout: float = 420.0
    crn_seed: int = 42
    gain_set: str = "phase_a"
    keep_fdm: bool = False        # keep the heavy sim_output FDM log per episode

    @property
    def specs(self):
        return gain_space.specs_for(self.gain_set)

    @property
    def runway_hdg(self):
        return self.base_cfg["mission"].get("runway_heading_deg", 285)

    @property
    def cruise_alt(self):
        u = self.base_cfg["mission"].get("ustol2", {})
        return u.get("cruise_alt_m", self.base_cfg["mission"].get("takeoff_alt_m", 100.0))


def evaluate_episode(gains: dict, scenario: scenarios.Scenario, instance_id: int,
                     run_dir: str, ctx: TunerContext, attempt: int = 0) -> dict:
    """-> flat episode record: identity, exit, cost layer fields, metrics.
    Never raises; infrastructure failures are flagged valid=False."""
    cand_id = gain_space.gain_hash(gains)
    parm    = gain_space.ensure_candidate_parm(run_dir, cand_id, gains,
                                               ctx.specs, ctx.base_parm)
    ucfg, _ = gain_space.split_by_delivery(gains, ctx.specs)
    cfg     = gain_space.apply_to_config(ctx.base_cfg, ucfg)

    perturbed = scenarios.perturbation_for(scenario, cfg, ctx.crn_seed)
    out_dir   = os.path.join(run_dir, "episodes", cand_id)
    os.makedirs(out_dir, exist_ok=True)

    t0 = time.time()
    result = mc_bridge.run_case(
        scenario.scen_id, f"tuner:{cand_id}:{scenario.scen_id}:d{scenario.fdm_draw}",
        instance_id, cfg, perturbed, out_dir, ctx.workspace, ctx.speedup,
        False,                       # save_plot
        ctx.hard_timeout, parm, scenario.wind,
        True,                        # dt_enabled — the mixer under tune
        scenario.fail_case,
        False, False, False)         # sim_parquet, trim_cols, keep_aux

    case_dir = os.path.join(out_dir, f"case_{scenario.scen_id:04d}")
    case_csv = os.path.join(case_dir, f"case_{scenario.scen_id:04d}_flight.csv")

    mission = ctx.base_cfg["mission"]
    metrics = episode_metrics.compute(case_csv, result, mission["home_lat"],
                                      mission["home_lon"], ctx.runway_hdg,
                                      ctx.cruise_alt)

    rec = {
        "cand_id": cand_id, "scen_id": scenario.scen_id,
        "scenario": scenario.label, "fail_case": scenario.fail_case,
        "wind_case": scenario.wind["wind_case"], "fdm_draw": scenario.fdm_draw,
        "tier": scenario.tier, "attempt": attempt,
        "exit_reason": result.get("exit_reason"),
        "land_result": result.get("land_result"),
        "valid": not objective.is_infra_failure(result),
        "wall_s": round(time.time() - t0, 1),
        "metrics": metrics,
    }
    if rec["valid"]:
        rec.update(objective.episode_cost(metrics, result))
    else:
        rec["cost"] = None

    _cleanup_episode(case_dir, ctx.keep_fdm)
    return rec


def _cleanup_episode(case_dir: str, keep_fdm: bool):
    """Delete the heavy FDM time-series (O(MB)/episode -> O(100GB)/campaign);
    keep the small flight CSV + result.json for audit/replay."""
    if keep_fdm:
        return
    for f in glob.glob(os.path.join(case_dir, "sim_output_*.csv")):
        try:
            os.remove(f)
        except OSError:
            pass
