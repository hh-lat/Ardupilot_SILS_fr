# gain_tuner — automatic gain tuning for the uSTOL aircraft

`gain_tuner` automatically searches for the best controller gains for the
uSTOL aircraft — including the custom differential-thrust (`UST_*`) and
EDF engine-out (`USTF_*`) gains that no stock autotuner knows about — by
flying thousands of simulated test flights and letting an optimizer learn
which gain values fly best.

It is designed to run **alongside** a Monte Carlo campaign
(`aws_monte_carlo_11_edf.py`): the campaign keeps producing statistics with
fixed gains, while the tuner runs its own separate SITL flights on spare
compute (a second AWS box, or a dev machine).

---

## 1. The idea in one paragraph

We can't compute the "best" gains with a formula, because the aircraft model
is a complicated nonlinear simulation with wind, turbulence, engine failures,
and randomized physics. But we *can* measure how good any particular gain set
is: load the gains into SITL, fly a full mission (takeoff → cruise → control
doublets → approach → flare landing), and score the flight. That turns tuning
into a **black-box optimization problem**: an optimizer proposes gain sets,
we fly them, we report back a score, and the optimizer proposes better ones.
Repeat for a few hundred rounds and the gains converge.

```
            ┌────────────────────────────────────────────────┐
            │                 OPTIMIZER (CMA-ES)              │
            │   "here are 16 candidate gain sets to try"      │
            └───────────────┬────────────────▲───────────────┘
                    ask()   │                │   tell(scores)
                            ▼                │
            ┌────────────────────────────────┴───────────────┐
            │                EPISODE EVALUATOR                │
            │  for each candidate: fly it in SITL under the   │
            │  SAME 48 test scenarios (failures × wind ×      │
            │  physics perturbations), score every flight,    │
            │  combine the 48 scores into one fitness number  │
            └────────────────────────────────────────────────┘
```

---

## 2. Vocabulary

| Term | Meaning here |
|---|---|
| **Gain set / candidate** | One complete assignment of values to the tuned parameters, e.g. `{UST_NDES_MAX: 23.1, UST_KRUD: 0.09, ...}` |
| **Episode** | One full simulated flight of the campaign mission profile with a candidate's gains loaded |
| **Scenario** | The conditions of one episode: which EDF failed × which wind × which randomized physics draw |
| **Cost `j`** | Score of one episode; **lower is better** (0 ≈ perfect flight, 1000 ≈ crashed on takeoff) |
| **Fitness `J`** | One number summarizing a candidate over all its 48 episodes (robust aggregate, see §5) |
| **Generation** | One optimizer round: 16 candidates proposed → evaluated → results told back |
| **CRN** | Common Random Numbers: every candidate flies the *identical* 48 scenarios, so score differences are caused by the gains, not luck |

---

## 3. What gets tuned

Defined in [`gain_space.py`](gain_space.py). Two presets:

- **`phase_a`** (7 dims, start here) — the custom gains only:
  `UST_DT_VLO`, `UST_DT_VHI` (via a ΔV trick, below), `UST_NDES_MAX`,
  `UST_KRUD`, `UST_DT_RLFF`, `USTF_ABST`, `USTF_MRED`
- **`full`** (24 dims) — phase_a **plus** roll/pitch rate PIDs
  (`RLL_RATE_P/I/D/FF`, `PTCH_RATE_*`, time constants), yaw
  (`YAW2SRV_DAMP`, `KFF_RDDRMIX`, `RUDD_DT_GAIN`) and TECS damping gains

Three encoding tricks worth understanding:

1. **Everything is normalized to [0,1]** before the optimizer sees it. The
   optimizer works in a unit box; `decode()` maps back to real units.
2. **Multiplicative gains search in log-space.** For a PID gain, going from
   0.1→0.4 and 0.1→0.025 should be "equally big" steps. Linear scaling would
   make the down-step tiny. So P/I/D/FF/time-constants are mapped through
   `log()` (range = baseline/4 … baseline×4).
3. **`UST_DT_VHI` is not tuned directly.** The physics requires VLO < VHI.
   Instead of rejecting invalid samples (which confuses the optimizer), we
   tune `VLO` and a positive gap `ΔV`, and decode `VHI = VLO + ΔV`. The
   constraint can then never be violated.

**How gains get into SITL** (two delivery routes, both already existing in
the campaign runner — we changed nothing):

- `UST_*` values → written into a copy of the mission config
  (`mission.ustol2` keys), which `_run_case()` param-sets after boot;
- everything else → a per-candidate merged `--defaults` .param file
  (base file with the tuned lines replaced).

---

## 4. One episode, step by step

[`episode.py`](episode.py) + [`mc_bridge.py`](mc_bridge.py). The tuner does
**not** reimplement any flight logic — it imports the campaign script as a
frozen library and calls its `_run_case()`:

```
gains ─┐
       ├─► merged .param file + patched config
scenario ─► wind params (SIM_WIND_*), EDF failure mask (USTF_MASK),
            physics perturbation file (LAT_MC_OVERRIDE_FILE)
       │
       ▼
  _run_case()  (imported, untouched, from aws_monte_carlo_11_edf.py)
       │   boots one SITL on its own TCP port, sets params, arms,
       │   flies: GROUND → CLIMB → CRUISE → YAW/ROLL DOUBLETS
       │          → APPROACH → FLARE → touchdown → ROLLOUT
       │   writes case_XXXX_flight.csv (~5 Hz telemetry) + result.json
       ▼
  episode_metrics.compute()   ─►  ~25 numbers (see §5)
  objective.episode_cost()    ─►  one cost j
```

Because the episode goes through the *same* code path as the Monte Carlo
campaign, a gain set that scores well here will behave identically when the
campaign validates it later.

`mc_bridge.py` also *asserts the exact signature* of `_run_case` at import —
if someone edits the campaign script incompatibly, the tuner fails loudly at
startup instead of silently mis-flying episodes.

---

## 5. Scoring: from telemetry to one number

### 5a. Episode metrics ([`episode_metrics.py`](episode_metrics.py))

From the flight CSV we extract, per phase:

- **Engine-out recovery** — peak yaw rate and bank during climb/cruise (the
  failure is active from boot), max deviation from the runway centreline,
  altitude sag during the doublets;
- **Tracking** — how well the aircraft followed the roll/yaw doublet
  commands (max/mean error, peak yaw rate, departures into RECOVER);
- **Landing** — touchdown sink rate / pitch / speed, ground roll, landing
  position error, approach & flare sink;
- **Control health** — how often the DT motor channels or throttle sit
  pinned at their limits (saturation), and oscillation detectors (reversal
  rate + high-frequency band power) that catch "gain buzz".

### 5b. Episode cost ([`objective.py`](objective.py))

Three **layers** that never overlap numerically, so the optimizer always
prioritizes the right thing (survive first, then respect hard limits, then
polish performance):

```
Did the flight end in a crash / timeout?
 ├─ YES → L1:  j = 1000 × (1 − progress)          → range ~150…1000
 │        progress grows with the phase reached
 │        (crash on climb ≈ 0.15, in flare ≈ 0.85)
 │        so even "all candidates crash" still gives a gradient
 └─ NO (landed)
     Did it break a hard limit? (td sink > 1 m/s, load > 2.5 g,
     ground roll > 15 m, repeated departures)
      ├─ YES → L2:  j = 100 + 20·(#breaches) + j_soft   → range 100…300
      └─ NO  → L3:  j = j_soft                          → range 0…10
```

`j_soft` is a weighted sum of the four metric blocks
(0.40·engine-out + 0.30·tracking + 0.30·landing + 0.15·control-health).
Every metric goes through one hinge function `phi(x; T_pass, T_fail)`:
cheap slope below the pass threshold (rewards margin), steep ramp between
pass and fail, **capped at 2** so one terrible metric can't be traded away
against ten small improvements. Thresholds come from the DASHBOARD failure
criteria; the ones marked *calibratable* in `objective.TH` should be re-set
from a baseline batch (median → T_pass, p90 → T_fail) and then frozen.

Because L2's floor (100) is far above L3's ceiling (~10), no amount of
tracking polish can ever buy back a hard-limit breach.

### 5c. Fitness over 48 scenarios ([`scenarios.py`](scenarios.py) + `aggregate()`)

One flight proves nothing — the aircraft must be robust across the whole
randomization envelope. Each candidate flies **K = 48 scenarios**:

- every failure scenario (healthy, outboard L/R, DT-channel L/R,
  aileron-channel L/R) gets ~6–7 slots (stratified);
- 6 wind conditions (calm → crosswind+downdraft) rotated across slots
  (Latin square) so every failure meets every wind;
- each slot has a fixed physics-perturbation seed (mass, inertia, aero
  derivatives ±…%). **Every candidate flies the identical 48 tuples** (CRN).
- 16 of the 48 slots redraw their physics each generation so the optimizer
  can't overfit one lucky set of draws.

The 48 costs collapse to one fitness with a **tail-focused** aggregate:

```
J = CVaR₀.₂₅ + 0.25 · mean       CVaR₀.₂₅ = mean of the WORST 12 of 48
```

Why not the plain mean? A candidate that flies 47 clean flights and crashes
once would look fine on average — but that one crash is exactly what we care
about. Why not the single worst? Too noisy (one unlucky seed decides
everything). CVaR of the worst quarter is the middle ground: robustness-
driven, but averaged enough to be stable.

### 5d. Not the gains' fault

Episodes that fail for infrastructure reasons (port never opened, MAVLink
didn't connect, arming refused) are retried once and then **excluded** —
never scored. Penalizing them would teach the optimizer nonsense. If >20% of
a generation is invalid, the run halts (that's an environment problem).

---

## 6. The optimizer ([`optimizer.py`](optimizer.py))

**CMA-ES** (Covariance Matrix Adaptation Evolution Strategy, `pip install
cma`): the standard workhorse for noisy continuous black-box problems with
5–100 dimensions. Intuition: it maintains a Gaussian "search cloud" over the
gain space, samples 16 candidates per generation, and moves/reshapes/shrinks
the cloud toward the candidates that ranked best. It uses only *rankings*,
never the raw cost values, which makes it robust to our wild cost scale
(0.1 vs 850) and to noise. It also learns *correlations* between gains (e.g.
"if P goes up, D must go up too") through its covariance matrix.

Two details that matter under noise:

- We ship **`xfavorite`** (the cloud's mean) as the final answer, not the
  single best-ever sample — the best-ever is usually just the luckiest.
- The whole optimizer state is checkpointed each generation. A killed run
  resumed with `--resume` re-asks the **exact same population** (verified),
  and already-flown episodes of the interrupted generation are recovered
  from the log instead of re-flown.

**Racing** (in [`runner.py`](runner.py)): most candidates in a generation
are bad, and we only need to know they're bottom-half. So stage 1 flies only
the 12 hardest scenarios; the bottom half of the population is eliminated
there, and only the survivors fly the remaining 36. Saves ~37% of episodes
per generation with almost no ranking damage.

Why not reinforcement learning? For "pick one gain vector per episode", RL
mathematically reduces to exactly this kind of evolution strategy — but with
weaker machinery. RL becomes genuinely more powerful only when the gains
should *change during flight* as a function of state (gain scheduling) —
that's a future extension (§10), and the `Optimizer` ask/tell interface plus
the `evals.jsonl` episode log are designed so it can drop in later.

---

## 7. Full flow

```
                          python3 -m gain_tuner tune
                                     │
             ┌───────────────────────▼───────────────────────┐
             │ startup: load config + base .param, hash them  │
             │ into run_meta.json (resume refuses on drift),  │
             │ build optimizer (or load checkpoint.pkl)       │
             └───────────────────────┬───────────────────────┘
                                     │  generation g = 0,1,2,…
   ┌─────────────────────────────────▼─────────────────────────────────┐
   │ build 48 CRN scenarios for gen g   (32 fixed + 16 fresh physics)  │
   │ xs = optimizer.ask()               (16 candidates in [0,1]^n)     │
   │ decode each → gains dict → merged .param + patched config         │
   │                                                                   │
   │  STAGE 1: all 16 candidates × 12 screen scenarios ──┐             │
   │     (episodes run in parallel: worker pool, one     │ 192 flights │
   │      SITL each on ports 7760+, watchdog-guarded)    │             │
   │  rank by stage-1 fitness → keep best 8              │             │
   │  STAGE 2: 8 survivors × remaining 36 scenarios ─────┘ 288 flights │
   │                                                                   │
   │  every episode → evals.jsonl   (cost, layer, metrics, scenario)   │
   │  per candidate → fitness J = CVaR₀.₂₅ + 0.25·mean                 │
   │  optimizer.tell(xs, J)                                            │
   │  checkpoint.pkl ▪ convergence.csv ▪ best/best_gains.{json,param}  │
   │  kill any zombie SITLs, next generation                           │
   └─────────────────────────────────┬─────────────────────────────────┘
                                     │  until --generations / --max-hours
                                     ▼
                 best/best_gains.param  +  best_ucfg.json
                                     │
                                     ▼
        validation: run the UNTOUCHED campaign runner twice with the
        same --seed (tuned vs baseline params) → paired comparison in
        the DASHBOARD.  Ship only if no failure×wind stratum regresses.
```

**File map**

| File | Responsibility |
|---|---|
| `mc_bridge.py` | only importer of the campaign script (frozen library + signature assert) |
| `gain_space.py` | which gains, bounds, [0,1] encode/decode, .param merge, config patch |
| `scenarios.py` | the 48 CRN scenario tuples per generation |
| `episode.py` | one flight: gains × scenario → metrics record |
| `episode_metrics.py` | flight CSV → ~25 objective ingredients |
| `objective.py` | episode cost layers + CVaR aggregation + thresholds table |
| `optimizer.py` | ask/tell protocol; CMA-ES + random-search fallback |
| `runner.py` | worker pool, racing, checkpoint/resume, logs, zombie sweep |
| `cli.py` | `eval` / `tune` / `report` subcommands |

---

## 8. Running it

```bash
cd Tools/autotest/monte_carlo
pip install cma

# 0) plumbing check: 4 baseline episodes (~4 min on a dev box)
python3 -m gain_tuner eval --profile smoke --workers 4 --speedup 10

# 1) noise-floor audit — how repeatable is the fitness number?
#    (do once, at the SAME speedup/worker load you will tune with)
python3 -m gain_tuner eval --repeat 3 --workers 40 --speedup 10

# 2) tune the custom DT gains overnight
python3 -m gain_tuner tune --gain-set phase_a --workers 57 --speedup 10 \
        --generations 150 --output-dir tuner_phaseA

#    interrupted? continue exactly where it stopped:
python3 -m gain_tuner tune ... --output-dir tuner_phaseA --resume

# 3) watch progress
python3 -m gain_tuner report tuner_phaseA
```

Output directory:

```
tuner_phaseA/
  run_meta.json      firmware/param/config hashes (resume safety)
  evals.jsonl        one line per episode — the full raw dataset
  candidates.jsonl   one line per candidate per generation
  convergence.csv    best/mean fitness, sigma, crash counts, best gains per gen
  checkpoint.pkl     optimizer state (atomic writes)
  best/              best_gains.json ▪ best_gains.param ▪ best_ucfg.json
  episodes/<cand>/   flight CSVs + result.json (heavy FDM logs deleted)
```

**Practical notes**

- SITL instance IDs **wrap at 256** in this build (`-I450` actually binds
  instance 194's port). The tuner uses base 200 and enforces
  `base + workers ≤ 256`. The MC campaign's base 100 stays disjoint as long
  as it uses <100 workers on the same box.
- `--speedup` trades fidelity for throughput. Tune and validate at the same
  speedup, and treat threshold breaches seen only at high speedup with
  suspicion (our smoke test showed ground-roll inflation at 10× on a loaded
  dev box).
- Adoption caveat: the campaign runner re-sets `UST_*` from the config's
  `mission.ustol2` block after boot, overriding the .param file — so when
  validating, copy `best_ucfg.json` into a copy of the config and pass it
  with `--config`, alongside `--params best_gains.param`.

---

## 9. Reading the results

`convergence.csv` tells the story of a run:

- `n_L1` (crashes) should fall toward 0 within the first ~20–40 generations;
- `n_L2` (hard-limit breaches) shrinks next;
- then `best_fitness` grinds down inside L3 territory (<10);
- `sigma` (search-cloud size) shrinking = the optimizer is converging;
  stop when the fitness of the recommendation is flat for ~25 generations.

The `rec_*` columns show where each gain is drifting — physically sanity-
check them (e.g. `UST_NDES_MAX` climbing to its bound means the yaw-moment
budget is the binding constraint, not the schedule).

---

## 10. How to improve it (roughly in order of value)

1. **Calibrate the placeholder thresholds.** Several `objective.TH` entries
   (`track_dev_max_m`, `landing_err_m`, `doublet_alt_sag_m`, …) are educated
   guesses. Fly ~500 baseline episodes, set T_pass = median and T_fail = p90,
   freeze, and only then start a real campaign. Mis-scaled thresholds skew
   which block dominates the soft cost.
2. **Add a validation monitor.** Every ~10 generations, fly the current
   recommendation on a *fresh* held-out scenario batch that never feeds the
   optimizer. A growing gap between training fitness and held-out fitness =
   overfitting to the 32 anchor scenarios → reshuffle anchors or enlarge the
   rotating tier. (The hooks exist: `build_eval_set(gen=...)` can key a
   disjoint seed range.)
3. **Smarter racing.** The 12-scenario screen is currently a fixed guess of
   "hard" tuples. Log which scenarios actually *separate* good from bad
   candidates (between-candidate cost variance per slot, all in
   `evals.jsonl`) and re-pick the screen every ~10 generations.
4. **Noise handling in the optimizer.** Wire up pycma's `NoiseHandler`
   (re-evaluates a few candidates, inflates sigma when rankings are
   unstable) — a drop-in around `tell()`.
5. **Surrogate pre-screening.** After a few thousand episodes, fit a cheap
   model (gradient-boosted trees) `ĵ(gains, scenario) → cost` on
   `evals.jsonl` and use it to discard obviously bad CMA samples before
   spending real episodes on them (~10–20% budget saving late in a run).
6. **Gain scheduling (the real "RL" upgrade).** A single fixed gain set is a
   compromise across the envelope. First measure the *headroom*: run three
   short specialized tunes ({healthy, outboard-failure, inboard-failure})
   and compare their fitness to the compromise. If the gap is >15%, tune a
   small scheduling law `gains = θ₀ + A·[dynamic pressure, thrust asymmetry,
   phase]` (~60 dims, still CMA-friendly). Only if that saturates is a
   neural policy trained with PPO/SAC (stable-baselines3, actions = bounded
   gain deltas at 1–5 Hz, dense shaped reward) worth its 10–100× episode
   budget and its much harder verification story.
7. **Multi-objective audit.** The block costs of every candidate are logged;
   plot engine-out vs landing vs control-health clouds after a campaign. If
   the chosen optimum sits on a steep trade-off cliff, rerun the last ~20
   generations with adjusted block weights — far cheaper than a true
   multi-objective (NSGA-II) campaign.
8. **True sideslip/heading metrics.** The 5 Hz flight CSV has no dedicated
   heading/beta channel; `track_dev` is a proxy. Reading one or two columns
   from the FDM's high-rate log before it is deleted would sharpen the
   engine-out block.
