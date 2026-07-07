"""Common-random-number (CRN) evaluation-set builder.

Every candidate in a generation flies the IDENTICAL list of scenario tuples
(failure x wind x FDM-perturbation seed), so fitness differences are
attributable to the gains, not to scenario luck — CMA-ES only needs ranks, and
CRN turns scenario noise into a paired comparison.

Structure (default profile, K=48):
  * failure axis, exact stratification: 7 failure scenarios + healthy, 6 slots
    each (48 total)
  * wind axis: 6 envelope-spanning conditions Latin-squared within each
    stratum (rotated by stratum index) and biased toward the hard cells
  * perturbation axis: one FDM draw per slot; draw 0 = nominal physics,
    draw d>0 = sample_parameters(default_rng([crn_seed, d]))

Tiers: 32 "anchor" slots keep their draw for the whole campaign; 16 "rotating"
slots get generation-keyed draws (still common across candidates within the
generation) so the optimizer can't overfit one lucky set of physics draws.
Stage-1 racing screen: `screen=True` marks the 12 historically hardest tuples
(engine-out + crosswind/downdraft) used to cut the bottom half of a population
before spending the remaining 36 episodes per candidate.
"""
from dataclasses import dataclass

from . import mc_bridge

# (case, horiz_speed, side, vert_speed) — mc_bridge.wind_params_for_case args.
# Ordered easy -> hard; the Latin square rotates this per failure stratum.
WIND_CONDITIONS = [
    ("none",       0.0, "right", 0.0),
    ("head",       1.5, "right", 0.0),
    ("tail",       1.5, "right", 0.0),
    ("cross",      2.0, "right", 0.0),
    ("down",       2.0, "right", 0.0),
    ("cross+down", 2.0, "right", 1.0),
]

# Failure strata: every scenario the campaign injects, healthy included.
FAILURE_ORDER = [
    "none", "outboard_port", "outboard_stbd",
    "dtch_port", "dtch_stbd", "ailch_port", "ailch_stbd",
]
SLOTS_PER_STRATUM = 6

# The stage-1 racing screen: hardest failure x wind combos (12 tuples).
_SCREEN = {
    ("outboard_port", "cross"), ("outboard_port", "cross+down"),
    ("outboard_stbd", "cross"), ("outboard_stbd", "cross+down"),
    ("dtch_port", "cross"), ("dtch_port", "down"),
    ("dtch_stbd", "cross"), ("dtch_stbd", "down"),
    ("ailch_port", "cross+down"), ("ailch_stbd", "cross+down"),
    ("none", "cross+down"), ("none", "tail"),
}

N_ROTATING = 16     # rotating-tier slot count (of the 48)


@dataclass(frozen=True)
class Scenario:
    scen_id: int
    fail_case: str
    wind: dict            # output of wind_params_for_case (SIM_WIND_* + metadata)
    fdm_draw: int         # 0 = nominal physics; >0 -> default_rng([crn_seed, draw])
    tier: str             # "anchor" | "rotating"
    screen: bool          # in the stage-1 racing screen

    @property
    def label(self):
        return f"s{self.scen_id:02d}:{self.fail_case}/{self.wind['wind_case']}/d{self.fdm_draw}"


def build_eval_set(runway_hdg: float, crn_seed: int, gen: int = 0,
                   profile: str = "default") -> list:
    """The K-episode evaluation set for one generation. Anchor draws depend
    only on crn_seed; rotating draws additionally on `gen` (common across the
    generation's candidates). Deterministic — no RNG state consumed here."""
    if profile == "smoke":            # 4 quick tuples for plumbing tests
        picks = [("none", 0), ("outboard_port", 3), ("dtch_stbd", 3), ("none", 5)]
        out = []
        for i, (fail, w) in enumerate(picks):
            case, spd, side, vspd = WIND_CONDITIONS[w]
            wind = mc_bridge.wind_params_for_case(case, runway_hdg, spd, side, vspd)
            out.append(Scenario(i, fail, wind, 0, "anchor", True))
        return out
    if profile != "default":
        raise ValueError(f"unknown scenario profile {profile!r}")

    scens = []
    sid = 0
    screened = set()
    K = 48
    for f_idx, fail in enumerate(FAILURE_ORDER):
        extra = 1 if f_idx < (K - SLOTS_PER_STRATUM * len(FAILURE_ORDER)) else 0
        for slot in range(SLOTS_PER_STRATUM + extra):
            case, spd, side, vspd = WIND_CONDITIONS[(slot + f_idx) % len(WIND_CONDITIONS)]
            wind = mc_bridge.wind_params_for_case(case, runway_hdg, spd, side, vspd)
            # Rotating tier: every 3rd slot id (16 of 48, spread across every
            # stratum) gets a generation-keyed physics draw. Anchor slots keep
            # a campaign-fixed draw: the first two slots of each stratum fly
            # nominal physics, the rest a fixed draw keyed by slot id.
            rotating = (sid % 3 == 2)
            if rotating:
                draw = 100_000 + gen * K + sid
            else:
                draw = 0 if slot < 2 else 1 + sid
            # Stage-1 screen: first occurrence of each hard (failure, wind) pair.
            is_screen = (fail, case) in _SCREEN and (fail, case) not in screened
            if is_screen:
                screened.add((fail, case))
            scens.append(Scenario(
                scen_id=sid, fail_case=fail, wind=wind, fdm_draw=draw,
                tier="rotating" if rotating else "anchor", screen=is_screen))
            sid += 1
    return scens


def perturbation_for(scenario: Scenario, cfg: dict, crn_seed: int) -> dict:
    """The FDM physical-parameter dict for a scenario (CRN-keyed)."""
    import numpy as np
    if scenario.fdm_draw == 0:
        return {k: v["nominal"] for k, v in cfg["params"].items()}
    rng = np.random.default_rng([crn_seed, scenario.fdm_draw])
    return mc_bridge.sample_parameters(cfg["params"], rng)
