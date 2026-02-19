=========================================
  LAT SITL — Monte Carlo Analysis Setup
=========================================

QUICK START
-----------
1. Build once (only needed once, or after C++ changes):
       cd /home/lat_avionics/Ardupilot_SITL_LATEST/Ardupilot_SILS
       ./waf plane

2. Edit configuration:
       Tools/autotest/monte_carlo/monte_carlo_config.json
       - Set "num_runs" (e.g. 100)
       - Set 3-sigma values for each parameter
       - Adjust mission profile (takeoff alt, circle radius, etc.)

3. Run:
       cd Tools/autotest/monte_carlo
       python3 monte_carlo_runner.py

   Options:
       --num-runs 10          Override number of cases
       --start-case 50        Resume from case 50
       --seed 42              Random seed (default 42)
       --config my.json       Use custom config file

4. Results:
       Logs_Simulations/MonteCarlo/
       ├── monte_carlo_summary.csv     ← all cases, params, status
       ├── Case0001_logdata/
       │   ├── monte_carlo_overrides.txt    ← exact params used
       │   ├── sim_output_*.csv             ← LAT FDM CSV log
       │   ├── *.BIN                        ← ArduPilot dataflash
       │   └── mav.tlog                     ← MAVProxy telemetry
       ├── Case0002_logdata/
       └── ...


HOW IT WORKS
------------
- Nominal params live in LAT_SIM_Runner.cpp (UNCHANGED)
- At startup, LAT_SIM_MonteCarlo.cpp reads monte_carlo_overrides.txt
- If that file doesn't exist → purely nominal run (zero impact)
- The Python script writes that file before each SITL launch
- After each run, logs are collected and SITL is killed


ADDING NEW PARAMETERS
---------------------
1. In monte_carlo_config.json: add entry with nominal + sigma_3
2. In LAT_SIM_MonteCarlo.cpp: add an  else if  line in apply_override()
3. Rebuild:  ./waf plane
   (Only needed once, not per Monte Carlo case)


FILES CREATED (no main files disturbed)
---------------------------------------
libraries/SITL/LAT_SIM_MonteCarlo.h       ← C++ header
libraries/SITL/LAT_SIM_MonteCarlo.cpp     ← Override file reader
Tools/autotest/monte_carlo/
    monte_carlo_config.json               ← User config
    monte_carlo_runner.py                 ← Main orchestrator
    README.txt                            ← This file

MINIMAL CHANGES TO EXISTING FILES
----------------------------------
libraries/SITL/LAT_SIM_Runner.cpp:
    Line  9:  #include "LAT_SIM_MonteCarlo.h"        (added)
    Line ~490: v_apply_monte_carlo_overrides();       (added at end of param define)
    These are no-ops when no override file is present.
