#ifndef LAT_SIM_MONTECARLO_H_
#define LAT_SIM_MONTECARLO_H_

/*
 * LAT_SIM_MonteCarlo.h
 * --------------------
 * Runtime parameter override system for Monte Carlo analysis.
 *
 * At SITL startup, after nominal params are set, this module reads
 * an override file (if present). If the file doesn't exist the nominal
 * values remain untouched — zero impact on normal operation.
 *
 * Override file location (hard-coded, absolute):
 *   /home/lat_avionics/Ardupilot_SITL_LATEST/Ardupilot_SILS/monte_carlo_overrides.txt
 *
 * File format (one param per line):
 *   param_name=value
 * Example:
 *   cg_x=1.045
 *   Ixx=15.2
 */

// Call AFTER nominal params are set in v_plane_param_define_eqx_v1_new_model().
// If the override file does not exist → no-op.
void v_apply_monte_carlo_overrides();

#endif // LAT_SIM_MONTECARLO_H_
