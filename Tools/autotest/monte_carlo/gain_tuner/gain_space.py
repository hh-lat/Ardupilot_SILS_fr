"""Gain-vector definition: which AP params are tuned, their bounds, encoding
to/from the optimizer's normalized [0,1]^n space, and the two delivery seams
into a SITL episode:

  delivery="ucfg"        set live by _run_case via config["mission"]["ustol2"]
                         keys (the UST_* set the runner param-sets after boot)
  delivery="param_file"  baked into a per-candidate merged --defaults .param
                         file (NAME,VALUE lines; covers any AP_Param)

Two encodings beyond plain linear lerp:
  log_scale     multiplicative gains (PID P/I/D/FF, TCONSTs, damping) search in
                log-space so a 4x-down and 4x-up step are symmetric
  UST_DT_DV     virtual dimension: VHI is optimized as VLO + dV (dV in [1,25])
                so the VLO<VHI ordering constraint can never be violated —
                no rejection/repair discontinuities in optimizer space
"""
import copy
import hashlib
import json
import math
import os
from dataclasses import dataclass, field

import numpy as np

# Virtual param: decodes to UST_DT_VHI = min(UST_DT_VLO + UST_DT_DV, VHI_CAP).
VIRTUAL_DV = "UST_DT_DV"
VHI_CAP    = 40.0
# Log-eps for shifted-log dims whose baseline is exactly 0 (PTCH_RATE_D).
_SHIFT_EPS = 1e-3


@dataclass(frozen=True)
class GainSpec:
    name: str                 # AP param name, or VIRTUAL_DV
    lo: float
    hi: float
    default: float
    delivery: str             # "ucfg" | "param_file" | "virtual"
    ucfg_key: str = None      # config["mission"]["ustol2"] key when delivery=="ucfg"
    log_scale: bool = False
    shifted: bool = False     # shifted log: z=0 decodes to exactly 0 (baseline-0 dims)


def _log_span(base, down=4.0, up=4.0):
    return base / down, base * up


# ---------------------------------------------------------------------------
#  The gain table. Bounds: AP_Param ranges for the custom DT set; baseline/4 ..
#  baseline*4 (log) for PID feedback terms; baseline/2 .. *2 for FF and time
#  constants (closer to physics — don't let FF mask bad feedback gains).
#  Baselines match ustol_sims/params_imp_v6_cruise.param + the runner's ucfg
#  defaults (aws_monte_carlo_11_edf._run_case).
# ---------------------------------------------------------------------------
DT_CORE = [
    GainSpec("UST_DT_VLO",   5.0, 20.0, 13.0, "ucfg", "ust_dt_vlo"),
    GainSpec(VIRTUAL_DV,     1.0, 25.0,  4.0, "virtual"),            # VHI = VLO + dV
    GainSpec("UST_NDES_MAX", 2.0, 50.0, 20.0, "ucfg", "ust_ndes_max"),
    GainSpec("UST_KRUD",     0.0,  1.0, 0.11, "ucfg", "ust_krud"),
    GainSpec("UST_DT_RLFF",  0.0,  1.0,  0.0, "ucfg", "ust_dt_rlff"),
]

FAILURE = [
    GainSpec("USTF_ABST", 0.0, 1.5, 0.5, "param_file"),
    GainSpec("USTF_MRED", 0.0, 1.0, 0.5, "param_file"),
]

RATE_PIDS = [
    GainSpec("RLL_RATE_P",      *_log_span(0.10),  0.10,  "param_file", log_scale=True),
    GainSpec("RLL_RATE_I",      *_log_span(0.04),  0.04,  "param_file", log_scale=True),
    GainSpec("RLL_RATE_D",      *_log_span(0.016), 0.016, "param_file", log_scale=True),
    GainSpec("RLL_RATE_FF",     *_log_span(1.65, 2, 2), 1.65, "param_file", log_scale=True),
    GainSpec("RLL2SRV_TCONST",  *_log_span(0.25, 2, 2), 0.25, "param_file", log_scale=True),
    GainSpec("PTCH_RATE_P",     *_log_span(0.65),  0.65,  "param_file", log_scale=True),
    GainSpec("PTCH_RATE_I",     *_log_span(0.34),  0.34,  "param_file", log_scale=True),
    GainSpec("PTCH_RATE_D",     _SHIFT_EPS, 0.1,   0.0,   "param_file", log_scale=True, shifted=True),
    GainSpec("PTCH_RATE_FF",    *_log_span(0.10, 2, 2), 0.10, "param_file", log_scale=True),
    GainSpec("PTCH2SRV_TCONST", *_log_span(0.30, 2, 2), 0.30, "param_file", log_scale=True),
    GainSpec("YAW2SRV_DAMP",    *_log_span(2.0),   2.0,   "param_file", log_scale=True),
    GainSpec("KFF_RDDRMIX",     0.0, 1.0,          1.0,   "param_file"),
    GainSpec("RUDD_DT_GAIN",    *_log_span(10.0, 2, 2), 10.0, "param_file", log_scale=True),
]

TECS = [
    GainSpec("TECS_TIME_CONST", *_log_span(8.0, 2, 2), 8.0, "param_file", log_scale=True),
    GainSpec("TECS_THR_DAMP",   *_log_span(0.7, 2, 2), 0.7, "param_file", log_scale=True),
    GainSpec("TECS_PTCH_DAMP",  *_log_span(0.6, 2, 2), 0.6, "param_file", log_scale=True),
    GainSpec("TECS_INTEG_GAIN", *_log_span(0.1),       0.1, "param_file", log_scale=True),
]

GAIN_SETS = {
    "dt_core": DT_CORE,
    "failure": FAILURE,
    "rate_pids": RATE_PIDS,
    "tecs": TECS,
    # Named campaign presets:
    "phase_a": DT_CORE + FAILURE,                       # ~7 dims — first overnight run
    "full":    DT_CORE + FAILURE + RATE_PIDS + TECS,    # ~24 dims — phase B
}


def specs_for(set_name: str):
    try:
        return GAIN_SETS[set_name]
    except KeyError:
        raise ValueError(f"unknown gain set {set_name!r}; choose from {sorted(GAIN_SETS)}")


# ---------------------------------------------------------------------------
#  encode/decode between {param_name: value} and the optimizer's [0,1]^n
# ---------------------------------------------------------------------------
def _to_unit(spec: GainSpec, v: float) -> float:
    if spec.log_scale:
        lo, hi = math.log(spec.lo), math.log(spec.hi)
        vv = math.log(max(v + (_SHIFT_EPS if spec.shifted else 0.0), spec.lo))
        return (vv - lo) / (hi - lo)
    return (v - spec.lo) / (spec.hi - spec.lo)


def _from_unit(spec: GainSpec, z: float) -> float:
    z = min(1.0, max(0.0, z))
    if spec.log_scale:
        lo, hi = math.log(spec.lo), math.log(spec.hi)
        v = math.exp(lo + z * (hi - lo))
        if spec.shifted:
            v = max(0.0, v - _SHIFT_EPS)
        return v
    return spec.lo + z * (spec.hi - spec.lo)


def defaults(specs) -> dict:
    """The baseline gain dict (actual AP params, VHI resolved from VLO+dV)."""
    return decode(encode_defaults(specs), specs)


def encode_defaults(specs) -> np.ndarray:
    return np.array([_to_unit(s, s.default) for s in specs], dtype=float)


def encode(gains: dict, specs) -> np.ndarray:
    """gains uses ACTUAL param names (UST_DT_VHI, not the dV virtual dim)."""
    out = []
    for s in specs:
        if s.name == VIRTUAL_DV:
            vlo = gains.get("UST_DT_VLO", 13.0)
            vhi = gains.get("UST_DT_VHI", vlo + s.default)
            out.append(_to_unit(s, vhi - vlo))
        else:
            out.append(_to_unit(s, gains.get(s.name, s.default)))
    return np.clip(np.array(out, dtype=float), 0.0, 1.0)


def decode(x, specs) -> dict:
    """[0,1]^n -> {actual_param_name: value}; resolves the dV virtual dim."""
    if len(x) != len(specs):
        raise ValueError(f"gain vector has {len(x)} dims, expected {len(specs)}")
    gains, dv = {}, None
    for z, s in zip(x, specs):
        v = _from_unit(s, float(z))
        if s.name == VIRTUAL_DV:
            dv = v
        else:
            gains[s.name] = v
    if dv is not None:
        gains["UST_DT_VHI"] = min(gains["UST_DT_VLO"] + dv, VHI_CAP)
    return gains


def gain_hash(gains: dict) -> str:
    """Stable short candidate id from the gain dict."""
    blob = json.dumps({k: round(float(v), 8) for k, v in sorted(gains.items())})
    return hashlib.sha1(blob.encode()).hexdigest()[:10]


# ---------------------------------------------------------------------------
#  Delivery: merged .param file + ucfg config injection
# ---------------------------------------------------------------------------
def split_by_delivery(gains: dict, specs):
    """-> (ucfg_dict {ucfg_key: value}, parmfile_dict {NAME: value})."""
    by_name = {s.name: s for s in specs}
    ucfg, parm = {}, {}
    for name, v in gains.items():
        spec = by_name.get(name)
        if name == "UST_DT_VHI":               # produced by the dV virtual dim
            ucfg["ust_dt_vhi"] = v
        elif spec is None:
            parm[name] = v                     # unknown -> param file (safe default)
        elif spec.delivery == "ucfg":
            ucfg[spec.ucfg_key] = v
        else:
            parm[name] = v
    return ucfg, parm


def make_param_file(base_parm: str, parm_gains: dict, out_path: str) -> str:
    """Merged --defaults file: base NAME,VALUE lines with tuned names replaced,
    missing names appended. Atomic write (SITL may boot from it concurrently)."""
    remaining = dict(parm_gains)
    lines = []
    with open(base_parm) as f:
        for line in f:
            raw = line.rstrip("\n")
            name = raw.split(",", 1)[0].strip()
            if name in remaining:
                lines.append(f"{name},{remaining.pop(name):.6g}")
            else:
                lines.append(raw)
    for name, v in sorted(remaining.items()):
        lines.append(f"{name},{v:.6g}")
    tmp = out_path + ".tmp"
    with open(tmp, "w") as f:
        f.write("\n".join(lines) + "\n")
    os.replace(tmp, out_path)
    return out_path


def apply_to_config(base_cfg: dict, ucfg_gains: dict) -> dict:
    """Deep-copied campaign config with the ucfg-delivered gains written into
    mission.ustol2 (the seam _run_case param-sets the UST_* values from)."""
    cfg = copy.deepcopy(base_cfg)
    cfg["mission"].setdefault("ustol2", {}).update(
        {k: float(v) for k, v in ucfg_gains.items()})
    return cfg


def ensure_candidate_parm(run_dir: str, cand_id: str, gains: dict, specs,
                          base_parm: str) -> str:
    """Idempotent per-candidate merged .param under run_dir/params/."""
    pdir = os.path.join(run_dir, "params")
    os.makedirs(pdir, exist_ok=True)
    out = os.path.join(pdir, f"{cand_id}.param")
    if not os.path.exists(out):
        _, parm_gains = split_by_delivery(gains, specs)
        make_param_file(base_parm, parm_gains, out)
    return out
