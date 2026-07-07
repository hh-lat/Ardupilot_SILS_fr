"""Pluggable ask/tell optimizer layer.

All optimizers speak normalized [0,1]^n vectors (gain_space encodes/decodes).
The protocol is deliberately RL-compatible: a future RL/gain-scheduling agent
implements ask() = sample candidates, tell() = update, and reuses the episode
evaluator, scenarios, and objective untouched.
"""
import pickle

import numpy as np


class Optimizer:
    """Protocol. ask() -> list of [0,1]^n vectors; tell(evals) with
    (vector, fitness) pairs, LOWER fitness = better."""

    def ask(self):
        raise NotImplementedError

    def tell(self, evals):
        raise NotImplementedError

    @property
    def recommendation(self):
        """Best current point (distribution mean for CMA — under noise the
        mean beats the noisy best-ever sample)."""
        raise NotImplementedError

    @property
    def should_stop(self):
        return False

    def state_bytes(self) -> bytes:
        return pickle.dumps(self.__dict__)

    def load_state_bytes(self, blob: bytes):
        self.__dict__.update(pickle.loads(blob))


class CMAESOptimizer(Optimizer):
    """Active CMA-ES via pycma (pip install cma). sigma0=0.25 of the
    normalized range; pycma's smooth boundary transform handles [0,1]."""

    def __init__(self, x0, popsize: int = 16, sigma0: float = 0.25, seed: int = 1):
        import cma                                   # lazy: only `tune` needs it
        self._cma = cma
        self.es = cma.CMAEvolutionStrategy(list(map(float, x0)), sigma0, {
            "popsize": popsize,
            "bounds": [0.0, 1.0],
            "CMA_active": True,
            "seed": seed,
            "verbose": -3,
        })

    def ask(self):
        return [np.asarray(x, dtype=float) for x in self.es.ask()]

    def tell(self, evals):
        xs = [list(map(float, x)) for x, _ in evals]
        fs = [float(f) for _, f in evals]
        self.es.tell(xs, fs)

    @property
    def recommendation(self):
        # xfavorite = distribution mean (phenotype) — under noise this beats
        # the noisy best-ever sample. Clip belt-and-braces to the box.
        return np.clip(np.asarray(self.es.result.xfavorite, dtype=float), 0.0, 1.0)

    @property
    def sigma(self):
        return float(self.es.sigma)

    @property
    def should_stop(self):
        return bool(self.es.stop())

    def state_bytes(self):
        return pickle.dumps(self.es)

    def load_state_bytes(self, blob):
        self.es = pickle.loads(blob)


class RandomSearchOptimizer(Optimizer):
    """Zero-dependency fallback + smoke-test path: Gaussian ball around the
    best point so far, shrinking slowly."""

    def __init__(self, x0, popsize: int = 16, sigma0: float = 0.25, seed: int = 1):
        self.rng = np.random.default_rng(seed)
        self.best_x = np.asarray(x0, dtype=float)
        self.best_f = None
        self.popsize = popsize
        self.sigma = sigma0
        self.gen = 0

    def ask(self):
        pop = [self.best_x.copy()] if self.gen == 0 else []
        while len(pop) < self.popsize:
            pop.append(np.clip(
                self.best_x + self.rng.normal(0, self.sigma, len(self.best_x)),
                0.0, 1.0))
        return pop

    def tell(self, evals):
        for x, f in evals:
            if f is not None and (self.best_f is None or f < self.best_f):
                self.best_f, self.best_x = float(f), np.asarray(x, dtype=float)
        self.gen += 1
        self.sigma = max(0.02, self.sigma * 0.97)

    @property
    def recommendation(self):
        return self.best_x

    @property
    def should_stop(self):
        return False


def make_optimizer(kind: str, x0, popsize: int, sigma0: float, seed: int) -> Optimizer:
    if kind == "cma":
        return CMAESOptimizer(x0, popsize, sigma0, seed)
    if kind == "random":
        return RandomSearchOptimizer(x0, popsize, sigma0, seed)
    raise ValueError(f"unknown optimizer {kind!r} (use cma|random)")
