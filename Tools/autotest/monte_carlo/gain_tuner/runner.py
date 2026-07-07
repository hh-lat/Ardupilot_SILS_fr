"""Generation loop: worker pool over SITL episodes, two-stage racing,
checkpoint/resume, JSONL logging.

SITL instance IDs WRAP AT 256 (uint8 in this build: -I450 binds the port of
instance 194), so usable ids are 0..255. The tuner defaults to base 200
(ports 7760+): disjoint from the MC campaign's --base-instance 100 for up to
100 campaign workers. On a bigger shared box, pick a base that keeps
[base, base+workers) clear of the campaign's range — EpisodePool enforces the
0..255 bound.
"""
import glob
import hashlib
import json
import os
import re
import signal
import subprocess
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime

import numpy as np

from . import episode, gain_space, objective, optimizer as opt_mod, scenarios

BASE_INSTANCE_DEFAULT = 200
INVALID_FRAC_HALT     = 0.20     # >20% infra-invalid episodes in a gen -> halt
RACE_KEEP_FRAC        = 0.5      # stage-1 survivors fraction
RACE_PESSIMISM        = 50.0     # added to eliminated candidates' stage-1 fitness

# ---------------------------------------------------------------------------
#  Worker-process globals (set once per worker by the pool initializer)
# ---------------------------------------------------------------------------
_g_queue = None
_g_ctx   = None
_g_dir   = None


def _init_worker(q, ctx, run_dir):
    global _g_queue, _g_ctx, _g_dir
    _g_queue, _g_ctx, _g_dir = q, ctx, run_dir


def _worker(task):
    gains, scenario, attempt = task
    inst = _g_queue.get()
    try:
        return episode.evaluate_episode(gains, scenario, inst, _g_dir, _g_ctx, attempt)
    finally:
        _g_queue.put(inst)


# ---------------------------------------------------------------------------
#  Episode pool: batch evaluation with one infra retry per episode
# ---------------------------------------------------------------------------
class EpisodePool:
    def __init__(self, ctx, run_dir, workers, base_instance=BASE_INSTANCE_DEFAULT):
        import multiprocessing
        if not (0 <= base_instance and base_instance + workers <= 256):
            raise ValueError(
                f"instance ids {base_instance}..{base_instance + workers - 1} out of "
                "range: SITL instance ids wrap at 256 (uint8) — the bound port would "
                "collide with another instance's. Use base+workers <= 256.")
        self.ctx, self.run_dir = ctx, run_dir
        self.workers = workers
        self.base_instance = base_instance
        self._manager = multiprocessing.Manager()
        q = self._manager.Queue()
        for w in range(workers):
            q.put(base_instance + w)
        self._q = q
        self._pool = ProcessPoolExecutor(
            max_workers=workers, initializer=_init_worker,
            initargs=(q, ctx, run_dir))

    def run_batch(self, jobs, on_done=None):
        """jobs: list of (gains_dict, Scenario). -> list of episode records
        (same order not guaranteed; each rec self-identifies). Infra failures
        are retried once, then returned with valid=False."""
        futs = {self._pool.submit(_worker, (g, s, 0)): (g, s) for g, s in jobs}
        recs = []
        while futs:
            for fut in as_completed(list(futs)):
                g, s = futs.pop(fut)
                try:
                    rec = fut.result()
                except Exception as exc:                    # worker died
                    rec = {"cand_id": gain_space.gain_hash(g), "scen_id": s.scen_id,
                           "scenario": s.label, "fail_case": s.fail_case,
                           "wind_case": s.wind["wind_case"], "fdm_draw": s.fdm_draw,
                           "tier": s.tier, "attempt": 0, "valid": False,
                           "exit_reason": f"worker_exception: {exc}", "cost": None,
                           "wall_s": 0.0, "metrics": {}}
                if not rec.get("valid") and rec.get("attempt", 0) == 0:
                    futs[self._pool.submit(_worker, (g, s, 1))] = (g, s)
                    continue
                recs.append(rec)
                if on_done:
                    on_done(rec)
        return recs

    def sweep_zombies(self):
        """Belt-and-braces: kill any arduplane SITL still alive on OUR
        instance ids (never touches the MC campaign's 100+ range)."""
        for w in range(self.workers):
            inst = self.base_instance + w
            try:
                out = subprocess.run(
                    ["pgrep", "-f", f"arduplane .*-I{inst}$"],
                    capture_output=True, text=True, timeout=10).stdout
            except Exception:
                continue
            for pid in out.split():
                try:
                    os.killpg(os.getpgid(int(pid)), signal.SIGKILL)
                except (OSError, ValueError):
                    pass

    def close(self):
        self._pool.shutdown(wait=False, cancel_futures=True)
        self.sweep_zombies()


# ---------------------------------------------------------------------------
#  Run directory: meta, logging, checkpointing
# ---------------------------------------------------------------------------
def _sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:16]


def write_run_meta(run_dir, ctx, args_dict):
    from . import mc_bridge
    meta = {
        "created": datetime.now().isoformat(),
        "binary_sha": _sha(str(mc_bridge.BINARY)) if mc_bridge.BINARY.exists() else None,
        "base_parm": ctx.base_parm, "base_parm_sha": _sha(ctx.base_parm),
        "gain_set": ctx.gain_set, "crn_seed": ctx.crn_seed,
        "speedup": ctx.speedup, "args": args_dict,
        "gains": [s.name for s in ctx.specs],
    }
    path = os.path.join(run_dir, "run_meta.json")
    if os.path.exists(path):
        old = json.load(open(path))
        for k in ("binary_sha", "base_parm_sha", "gain_set", "crn_seed"):
            if old.get(k) != meta[k] and not args_dict.get("force"):
                raise RuntimeError(
                    f"resume mismatch on {k!r}: run was {old.get(k)}, now {meta[k]}. "
                    "The firmware/params/gain-set changed under the run — evaluations "
                    "would not be comparable. Pass --force to override.")
        return old
    with open(path, "w") as f:
        json.dump(meta, f, indent=2)
    return meta


def append_jsonl(path, rec):
    with open(path, "a") as f:
        f.write(json.dumps(rec) + "\n")


def load_gen_evals(run_dir, gen):
    """Completed (cand_id, scen_id) -> rec for a generation (mid-gen resume)."""
    done = {}
    path = os.path.join(run_dir, "evals.jsonl")
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                try:
                    r = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if r.get("gen") == gen and r.get("valid"):
                    done[(r["cand_id"], r["scen_id"])] = r
    return done


def save_checkpoint(run_dir, opt, gen):
    # NOTE: pycma samples from the global np.random RandomState; the pickle
    # captures its state, so a restarted run re-asks the exact pre-crash
    # population (verified) PROVIDED the main process draws nothing from
    # np.random.* (legacy API) between checkpoint and the next ask. Scenario
    # perturbations use np.random.default_rng (independent), so keep it that way.
    blob = opt.state_bytes()
    tmp = os.path.join(run_dir, "checkpoint.pkl.tmp")
    with open(tmp, "wb") as f:
        f.write(blob)
    os.replace(tmp, os.path.join(run_dir, "checkpoint.pkl"))
    with open(os.path.join(run_dir, "checkpoint.json"), "w") as f:
        json.dump({"gen": gen, "saved": datetime.now().isoformat()}, f)


def load_checkpoint(run_dir, opt):
    cj = os.path.join(run_dir, "checkpoint.json")
    cp = os.path.join(run_dir, "checkpoint.pkl")
    if not (os.path.exists(cj) and os.path.exists(cp)):
        return 0
    with open(cp, "rb") as f:
        opt.load_state_bytes(f.read())
    return json.load(open(cj))["gen"] + 1


def update_best(run_dir, ctx, opt):
    gains = gain_space.decode(opt.recommendation, ctx.specs)
    best = os.path.join(run_dir, "best")
    os.makedirs(best, exist_ok=True)
    with open(os.path.join(best, "best_gains.json"), "w") as f:
        json.dump({k: round(float(v), 6) for k, v in sorted(gains.items())}, f, indent=2)
    ucfg, parm = gain_space.split_by_delivery(gains, ctx.specs)
    gain_space.make_param_file(ctx.base_parm, parm,
                               os.path.join(best, "best_gains.param"))
    with open(os.path.join(best, "best_ucfg.json"), "w") as f:
        json.dump({k: round(float(v), 6) for k, v in sorted(ucfg.items())}, f, indent=2)
    return gains


# ---------------------------------------------------------------------------
#  Candidate evaluation with two-stage racing
# ---------------------------------------------------------------------------
def _fitness_of(recs, alpha, mean_w):
    return objective.aggregate([r.get("cost") for r in recs if r.get("valid")],
                               alpha=alpha, mean_w=mean_w)


def evaluate_generation(pool, ctx, run_dir, gen, xs, eval_set, alpha=0.25,
                        mean_w=0.25, racing=True, log=print):
    """Evaluate a population. -> (fitness list aligned with xs, gen stats)."""
    cands, _by_id = [], {}
    for x in xs:
        gains = gain_space.decode(x, ctx.specs)
        cid = gain_space.gain_hash(gains)
        # Duplicate candidates (possible after decode rounding) share one recs
        # list so both get the episodes flown once for that gain set.
        recs = _by_id.setdefault(cid, [])
        cands.append({"x": x, "gains": gains, "cand_id": cid, "recs": recs})

    done = load_gen_evals(run_dir, gen)          # mid-generation resume
    evals_path = os.path.join(run_dir, "evals.jsonl")
    n_run = [0]

    def _log_rec(stage):
        def cb(rec):
            rec["gen"], rec["stage"] = gen, stage
            append_jsonl(evals_path, rec)
            n_run[0] += 1
        return cb

    def _run(cand_list, scens, stage):
        # Unique candidates only: duplicates share a recs list, so flying (and
        # cache-appending) once per cand_id covers all copies.
        uniq = list({c["cand_id"]: c for c in cand_list}.values())
        jobs, cached = [], 0
        for c in uniq:
            for s in scens:
                hit = done.get((c["cand_id"], s.scen_id))
                if hit is not None:
                    c["recs"].append(hit)
                    cached += 1
                else:
                    jobs.append((c["gains"], s))
        if cached:
            log(f"    gen {gen} stage {stage}: {cached} episodes recovered from log")
        by_cand = {c["cand_id"]: c for c in uniq}
        for rec in pool.run_batch(jobs, on_done=_log_rec(stage)):
            by_cand[rec["cand_id"]]["recs"].append(rec)

    screen = [s for s in eval_set if s.screen]
    rest   = [s for s in eval_set if not s.screen]

    if racing and len(screen) >= 4 and len(cands) >= 4:
        _run(cands, screen, 1)
        ranked = sorted(cands, key=lambda c: (
            _fitness_of(c["recs"], 0.5, mean_w)["fitness"] or float("inf")))
        keep = max(2, int(np.ceil(len(cands) * RACE_KEEP_FRAC)))
        survivors, eliminated = ranked[:keep], ranked[keep:]
        log(f"    gen {gen} racing: {len(survivors)}/{len(cands)} advance to stage 2")
        _run(survivors, rest, 2)
    else:
        _run(cands, eval_set, 1)
        survivors, eliminated = cands, []

    fitness, stats = [], []
    max_finisher = 0.0
    for c in cands:
        agg = _fitness_of(c["recs"], alpha, mean_w)
        c["agg"] = agg
        if c in survivors and agg["fitness"] is not None:
            max_finisher = max(max_finisher, agg["fitness"])
    for c in cands:
        f = c["agg"]["fitness"]
        if f is None:
            f = 2000.0                                    # fully-invalid candidate
        elif c not in survivors:
            # eliminated: rank strictly below every finisher (total order for tell)
            f = max(f + RACE_PESSIMISM, max_finisher + RACE_PESSIMISM)
        fitness.append(f)
        stats.append({"gen": gen, "cand_id": c["cand_id"], "fitness": round(f, 3),
                      **{k: (round(v, 3) if isinstance(v, float) else v)
                         for k, v in c["agg"].items()},
                      "survivor": c in survivors,
                      "gains": {k: round(float(v), 5) for k, v in sorted(c["gains"].items())}})

    all_recs = [r for c in cands for r in c["recs"]]
    n_invalid = sum(1 for r in all_recs if not r.get("valid"))
    if all_recs and n_invalid / len(all_recs) > INVALID_FRAC_HALT:
        raise RuntimeError(
            f"{n_invalid}/{len(all_recs)} episodes infra-invalid in gen {gen} — "
            "environment problem (ports/binary/params), halting rather than "
            "corrupting the fitness landscape.")

    cand_path = os.path.join(run_dir, "candidates.jsonl")
    for s in stats:
        append_jsonl(cand_path, s)
    return fitness, {"n_episodes_run": n_run[0], "n_invalid": n_invalid,
                     "layer_counts": _layer_counts(all_recs)}


def _layer_counts(recs):
    out = {}
    for r in recs:
        k = r.get("layer", "invalid" if not r.get("valid") else "?")
        out[k] = out.get(k, 0) + 1
    return out


# ---------------------------------------------------------------------------
#  The tune loop
# ---------------------------------------------------------------------------
def tune(ctx, run_dir, workers, generations, popsize=16, sigma0=0.25,
         opt_kind="cma", seed=1, alpha=0.25, mean_w=0.25, racing=True,
         base_instance=BASE_INSTANCE_DEFAULT, resume=False, force=False,
         max_hours=None, log=print):
    os.makedirs(run_dir, exist_ok=True)
    write_run_meta(run_dir, ctx, {"workers": workers, "popsize": popsize,
                                  "opt": opt_kind, "alpha": alpha,
                                  "racing": racing, "force": force})
    x0 = gain_space.encode_defaults(ctx.specs)
    opt = opt_mod.make_optimizer(opt_kind, x0, popsize, sigma0, seed)
    gen0 = load_checkpoint(run_dir, opt) if resume else 0
    if resume:
        log(f"  resume: continuing at generation {gen0}")

    conv_path = os.path.join(run_dir, "convergence.csv")
    param_names = sorted(gain_space.decode(x0, ctx.specs))
    if not os.path.exists(conv_path):
        with open(conv_path, "w") as f:
            f.write(",".join(["gen", "best_fitness", "mean_fitness", "sigma",
                              "n_episodes", "n_invalid", "n_L1", "n_L2", "n_L3"]
                             + [f"rec_{n}" for n in param_names]) + "\n")

    pool = EpisodePool(ctx, run_dir, workers, base_instance)
    t0 = time.time()
    try:
        for gen in range(gen0, generations):
            if max_hours and (time.time() - t0) / 3600.0 > max_hours:
                log(f"  max-hours budget reached at gen {gen}")
                break
            eval_set = scenarios.build_eval_set(ctx.runway_hdg, ctx.crn_seed, gen)
            xs = opt.ask()
            t_gen = time.time()
            fitness, gstats = evaluate_generation(
                pool, ctx, run_dir, gen, xs, eval_set, alpha, mean_w, racing, log)
            opt.tell(list(zip(xs, fitness)))
            save_checkpoint(run_dir, opt, gen)
            rec_gains = update_best(run_dir, ctx, opt)
            pool.sweep_zombies()

            lc = gstats["layer_counts"]
            sigma = getattr(opt, "sigma", float("nan"))
            with open(conv_path, "a") as f:
                f.write(",".join(map(str, [
                    gen, round(min(fitness), 3),
                    round(float(np.mean(fitness)), 3), round(sigma, 4),
                    gstats["n_episodes_run"], gstats["n_invalid"],
                    lc.get("L1", 0), lc.get("L2", 0), lc.get("L3", 0)]
                    + [round(rec_gains[n], 5) for n in param_names])) + "\n")
            log(f"  gen {gen:3d}: best {min(fitness):8.2f}  "
                f"mean {np.mean(fitness):8.2f}  sigma {sigma:.3f}  "
                f"episodes {gstats['n_episodes_run']}  "
                f"L1/L2/L3 {lc.get('L1',0)}/{lc.get('L2',0)}/{lc.get('L3',0)}  "
                f"({time.time()-t_gen:.0f}s)")
            if opt.should_stop:
                log("  optimizer signalled convergence — stopping")
                break
    finally:
        pool.close()
    log(f"\n  done. best gains -> {os.path.join(run_dir, 'best')}")
    return update_best(run_dir, ctx, opt)


# ---------------------------------------------------------------------------
#  One-shot candidate evaluation (the `eval` CLI — hand-tuning + noise audit)
# ---------------------------------------------------------------------------
def evaluate(ctx, run_dir, workers, gains, profile="default", repeat=1,
             alpha=0.25, mean_w=0.25, base_instance=BASE_INSTANCE_DEFAULT,
             log=print):
    os.makedirs(run_dir, exist_ok=True)
    eval_set = scenarios.build_eval_set(ctx.runway_hdg, ctx.crn_seed, 0, profile)
    pool = EpisodePool(ctx, run_dir, workers, base_instance)
    evals_path = os.path.join(run_dir, "evals.jsonl")
    out = []
    try:
        for rep in range(repeat):
            def cb(rec, rep=rep):
                rec["gen"], rec["stage"], rec["repeat"] = -1, 0, rep
                append_jsonl(evals_path, rec)
            recs = pool.run_batch([(gains, s) for s in eval_set], on_done=cb)
            agg = objective.aggregate(
                [r.get("cost") for r in recs if r.get("valid")], alpha, mean_w)
            out.append({"repeat": rep, "agg": agg, "recs": recs})
            if agg["fitness"] is None:
                reasons = {}
                for r in recs:
                    reasons[r.get("exit_reason")] = reasons.get(r.get("exit_reason"), 0) + 1
                log(f"  repeat {rep}: NO VALID EPISODES ({len(recs)} flown) — "
                    f"exit reasons: {reasons}")
            else:
                log(f"  repeat {rep}: fitness {agg['fitness']:.2f}  cvar {agg['cvar']:.2f}  "
                    f"mean {agg['mean']:.2f}  n {agg['n']}/{len(eval_set)}")
    finally:
        pool.close()
    if repeat > 1:
        fits = [o["agg"]["fitness"] for o in out if o["agg"]["fitness"] is not None]
        if fits:
            log(f"\n  noise floor over {repeat} repeats: "
                f"spread {max(fits)-min(fits):.2f} ({np.std(fits):.2f} std) "
                f"around {np.mean(fits):.2f}")
    return out
