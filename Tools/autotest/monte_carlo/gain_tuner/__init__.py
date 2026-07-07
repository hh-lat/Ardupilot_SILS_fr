"""gain_tuner — automatic gain tuning alongside the EDF Monte Carlo campaigns.

Runs its own SITL episodes (reusing the aws_monte_carlo_11_edf machinery via
mc_bridge, never touching the campaign script) and drives a pluggable black-box
optimizer (CMA-ES first; RL can drop in behind the same ask/tell protocol) over
the custom UST_*/USTF_* differential-thrust gains plus the standard autopilot
gains. See Tools/autotest/monte_carlo/gain_tuner/README.md.
"""
