"""python3 -m gain_tuner {eval|tune|report}

Run from Tools/autotest/monte_carlo/ (or anywhere — paths resolve through the
campaign script's own SCRIPT_DIR/WORKSPACE). Uses instance IDs 200+ (ports
7760+; SITL instance ids wrap at 256, so 200+workers must stay <= 256) —
disjoint from an MC campaign on --base-instance 100 with <100 workers.

  eval    fly one gain set over the CRN evaluation set (hand-tuning + the
          --repeat noise-floor audit that gates trusting CMA rankings)
  tune    the CMA-ES generation loop (checkpointed; --resume continues)
  report  print convergence tail + current best gains of a run dir
"""
import argparse
import json
import os
import sys
from datetime import datetime

from . import gain_space, mc_bridge, runner
from .episode import TunerContext


def _add_common(p):
    p.add_argument("--config", default=str(mc_bridge.DEFAULT_CFG))
    p.add_argument("--params", default=None,
                   help="base defaults .param (default: campaign auto-resolve)")
    p.add_argument("--gain-set", default="phase_a",
                   choices=sorted(gain_space.GAIN_SETS),
                   help="which gains are tuned (phase_a = custom DT+failure set)")
    p.add_argument("--workers", type=int, default=8)
    p.add_argument("--speedup", type=int, default=10)
    p.add_argument("--hard-timeout", type=float, default=420.0)
    p.add_argument("--crn-seed", type=int, default=42,
                   help="common-random-numbers seed (fixed for a whole campaign)")
    p.add_argument("--base-instance", type=int, default=runner.BASE_INSTANCE_DEFAULT)
    p.add_argument("--output-dir", default=None)
    p.add_argument("--keep-fdm", action="store_true",
                   help="keep each episode's heavy sim_output FDM log")


def _ctx_from(args):
    base_parm = os.path.abspath(args.params) if args.params \
        else mc_bridge.resolve_defaults_parm()
    return TunerContext(
        base_cfg=mc_bridge.load_config(args.config), base_parm=base_parm,
        workspace=str(mc_bridge.WORKSPACE), speedup=args.speedup,
        hard_timeout=args.hard_timeout, crn_seed=args.crn_seed,
        gain_set=args.gain_set, keep_fdm=args.keep_fdm)


def _run_dir(args, tag):
    if args.output_dir:
        return os.path.abspath(args.output_dir)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return str(mc_bridge.SCRIPT_DIR / f"tuner_{ts}_{tag}")


def main(argv=None):
    ap = argparse.ArgumentParser(prog="gain_tuner", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    pe = sub.add_parser("eval", help="evaluate one gain set over the eval set")
    _add_common(pe)
    pe.add_argument("--gains", default=None,
                    help="JSON file {PARAM: value}; omit for the baseline gains")
    pe.add_argument("--profile", default="default", choices=["default", "smoke"])
    pe.add_argument("--repeat", type=int, default=1,
                    help=">1: repeat the whole set to measure the fitness noise floor")

    pt = sub.add_parser("tune", help="run the optimizer loop")
    _add_common(pt)
    pt.add_argument("--generations", type=int, default=150)
    pt.add_argument("--popsize", type=int, default=16)
    pt.add_argument("--sigma0", type=float, default=0.25)
    pt.add_argument("--optimizer", default="cma", choices=["cma", "random"])
    pt.add_argument("--seed", type=int, default=1, help="optimizer RNG seed")
    pt.add_argument("--alpha", type=float, default=0.25, help="CVaR tail fraction")
    pt.add_argument("--mean-weight", type=float, default=0.25)
    pt.add_argument("--no-racing", action="store_true",
                    help="disable two-stage racing (all candidates fly all K)")
    pt.add_argument("--resume", action="store_true")
    pt.add_argument("--force", action="store_true",
                    help="resume despite binary/params hash mismatch")
    pt.add_argument("--max-hours", type=float, default=None)

    pr = sub.add_parser("report", help="summarize a tuner run dir")
    pr.add_argument("run_dir")
    pr.add_argument("--tail", type=int, default=10)

    args = ap.parse_args(argv)

    if args.cmd == "report":
        return report(args.run_dir, args.tail)

    if not mc_bridge.BINARY.exists():
        print(f"ERROR: SITL binary not found: {mc_bridge.BINARY}\n"
              "       Build with ./waf plane first.")
        return 1
    ctx = _ctx_from(args)

    if args.cmd == "eval":
        if args.gains:
            with open(args.gains) as f:
                gains = {k: float(v) for k, v in json.load(f).items()}
        else:
            gains = gain_space.defaults(ctx.specs)
            print("  (no --gains: evaluating the baseline gain set)")
        run_dir = _run_dir(args, "eval")
        print(f"  gain set : {args.gain_set} ({len(ctx.specs)} dims)")
        print(f"  profile  : {args.profile}   repeat {args.repeat}")
        print(f"  output   : {run_dir}\n")
        runner.evaluate(ctx, run_dir, args.workers, gains, args.profile,
                        args.repeat, base_instance=args.base_instance)
        return 0

    if args.cmd == "tune":
        run_dir = _run_dir(args, args.gain_set)
        print(f"  gain set : {args.gain_set} ({len(ctx.specs)} dims)")
        print(f"  optimizer: {args.optimizer}  pop {args.popsize}  "
              f"sigma0 {args.sigma0}  CVaR alpha {args.alpha}")
        print(f"  workers  : {args.workers}  speedup {args.speedup}x  "
              f"instances {args.base_instance}+")
        print(f"  output   : {run_dir}\n")
        best = runner.tune(
            ctx, run_dir, args.workers, args.generations, args.popsize,
            args.sigma0, args.optimizer, args.seed, args.alpha, args.mean_weight,
            racing=not args.no_racing, base_instance=args.base_instance,
            resume=args.resume, force=args.force, max_hours=args.max_hours)
        print("\n  recommended gains:")
        for k, v in sorted(best.items()):
            print(f"    {k:<18} {v:.5g}")
        return 0
    return 1


def report(run_dir, tail):
    conv = os.path.join(run_dir, "convergence.csv")
    if os.path.exists(conv):
        lines = open(conv).read().strip().splitlines()
        print(f"  {lines[0]}")
        for ln in lines[-tail:]:
            print(f"  {ln}")
    else:
        print("  (no convergence.csv yet)")
    best = os.path.join(run_dir, "best", "best_gains.json")
    if os.path.exists(best):
        print("\n  best gains:")
        for k, v in sorted(json.load(open(best)).items()):
            print(f"    {k:<18} {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
