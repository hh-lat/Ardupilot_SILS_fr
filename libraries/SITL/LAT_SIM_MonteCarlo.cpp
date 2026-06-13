/*
 * LAT_SIM_MonteCarlo.cpp
 * ----------------------
 * Reads a simple key=value text file at SITL startup and overrides
 * the corresponding vehcle struct fields.  If the file does not
 * exist the function is a silent no-op, so normal (non-Monte-Carlo)
 * runs are completely unaffected.
 *
 * To add a new perturbable parameter:
 *   1. Add an  else if (strcmp(key, "NEW_PARAM") == 0) { vehcle.NEW_PARAM = val; }
 *      block inside apply_override().
 *   2. Add "NEW_PARAM" to the Python config (monte_carlo_config.json).
 *   That's it — no rebuild of the override reader is needed for the
 *   Python side; only for mapping new C++ fields.
 */

#include "LAT_SIM_MonteCarlo.h"
#include "LAT_SIM_Runner.h"
#include "LAT_SIM_rotor_dynamics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static const char MC_OVERRIDE_FILE_DEFAULT[] =
    "/home/lag/SITL_Workspace/Ardupilot_SILS/monte_carlo_overrides.txt";

// Set true when the dimensional CG (cg_x) is overridden, so the legacy
// landing-gear geometry recompute only runs on that (Equinox) path and does
// not clobber the explicit uSTOL gear geometry on a uSTOL run.
static bool s_cg_x_overridden = false;

// ---- Map a key string to the correct vehcle field ----
static void apply_override(const char* key, float val)
{
    // CG
    if      (strcmp(key, "cg_x") == 0)        { vehcle.cg_x = val; s_cg_x_overridden = true; }
    else if (strcmp(key, "cg_z") == 0)        { vehcle.cg_z = val; }
    // uSTOL non-dimensional aero CG (x_cg/c) — drives the pitch moment;
    // distinct from the dimensional cg_x above (which is landing-gear geometry).
    else if (strcmp(key, "cg_x_cg_c") == 0)   { vehcle.cg.x_cg_c = val; }
    // Inertias
    else if (strcmp(key, "Ixx") == 0)         { vehcle.Ixx = val; }
    else if (strcmp(key, "Iyy") == 0)         { vehcle.Iyy = val; }
    else if (strcmp(key, "Izz") == 0)         { vehcle.Izz = val; }
    else if (strcmp(key, "Ixz") == 0)         { vehcle.Ixz = val; }
    // Mass
    else if (strcmp(key, "mass") == 0)        { vehcle.mass = val; }
    // Lift coefficients
    else if (strcmp(key, "CL_0") == 0)        { vehcle.CL_0 = val; }
    else if (strcmp(key, "CL_alpha") == 0)    { vehcle.CL_alpha = val; }
    else if (strcmp(key, "CL_delta_e") == 0)  { vehcle.CL_delta_e = val; }
    else if (strcmp(key, "CL_q") == 0)        { vehcle.CL_q = val; }
    // Mass & atmosphere
    else if (strcmp(key, "rho") == 0)          { vehcle.rho = val; }
    // Drag coefficients
    else if (strcmp(key, "CD_0") == 0)          { vehcle.CD_0 = val; }
    else if (strcmp(key, "CD_alpha") == 0)      { vehcle.CD_alpha = val; }
    else if (strcmp(key, "CD_delta_e") == 0)    { vehcle.CD_delta_e = val; }
    else if (strcmp(key, "CD_delta_f") == 0)    { vehcle.CD_delta_f = val; }
    else if (strcmp(key, "CD_delta_e2") == 0)   { vehcle.CD_delta_e2 = val; }
    // Pitch-moment coefficients
    else if (strcmp(key, "Cm_0") == 0)          { vehcle.Cm_0 = val; }
    else if (strcmp(key, "Cm_alpha") == 0)      { vehcle.Cm_alpha = val; }
    else if (strcmp(key, "Cm_delta_e") == 0)    { vehcle.Cm_delta_e = val; }
    else if (strcmp(key, "Cm_q") == 0)          { vehcle.Cm_q = val; }
    else if (strcmp(key, "Cm_Cmu") == 0)        { vehcle.Cm_Cmu = val; }
    else if (strcmp(key, "Cm_delta_f") == 0)    { vehcle.Cm_delta_f = val; }
    else if (strcmp(key, "Cm_beta2") == 0)      { vehcle.Cm_beta2 = val; }
    // Side-force coefficients
    else if (strcmp(key, "CY_beta") == 0)     { vehcle.CY_beta = val; }
    else if (strcmp(key, "CY_delta_r") == 0)  { vehcle.CY_delta_r = val; }
    // Roll-moment coefficients
    else if (strcmp(key, "Cl_0") == 0)          { vehcle.Cl_0 = val; }
    else if (strcmp(key, "Cl_beta") == 0)       { vehcle.Cl_beta = val; }
    else if (strcmp(key, "Cl_delta_r") == 0)    { vehcle.Cl_delta_r = val; }
    else if (strcmp(key, "Cl_delta_aL") == 0)   { vehcle.Cl_delta_aL = val; }
    else if (strcmp(key, "Cl_delta_aR") == 0)   { vehcle.Cl_delta_aR = val; }
    else if (strcmp(key, "Cl_p") == 0)          { vehcle.Cl_p = val; }
    else if (strcmp(key, "Cl_r") == 0)          { vehcle.Cl_r = val; }
    // Yaw-moment coefficients
    else if (strcmp(key, "Cn_beta") == 0)     { vehcle.Cn_beta = val; }
    else if (strcmp(key, "Cn_delta_r") == 0)  { vehcle.Cn_delta_r = val; }
    else if (strcmp(key, "Cn_p") == 0)        { vehcle.Cn_p = val; }
    else if (strcmp(key, "Cn_r") == 0)        { vehcle.Cn_r = val; }
    // Geometry
    else if (strcmp(key, "s") == 0)           { vehcle.s = val; }
    else if (strcmp(key, "b") == 0)           { vehcle.b = val; }
    else if (strcmp(key, "c") == 0)             { vehcle.c = val; }
    // Motor max thrust (applied to ALL motors)
    else if (strcmp(key, "max_thrust") == 0) {
        for (int i = 0; i < s_motor_manager.num_motors; i++) {
            s_motor[i].max_thrust = val;
        }
    }
    // ======================================================================
    //  uSTOL_v1 (PLANE_USTOL_V1) component parameters
    // ======================================================================
    // Shared physical (additional)
    else if (strcmp(key, "t_by_c") == 0)         { vehcle.t_by_c = val; }
    else if (strcmp(key, "s_blown") == 0)        { vehcle.s_blown = val; }
    // Wing
    else if (strcmp(key, "wing_lambda_b") == 0)  { vehcle.wing.lambda_b = val; }
    else if (strcmp(key, "wing_S_f") == 0)       { vehcle.wing.S_f = val; }
    else if (strcmp(key, "wing_S_a") == 0)       { vehcle.wing.S_a = val; }
    else if (strcmp(key, "wing_k_fit") == 0)     { vehcle.wing.k_fit = val; }
    else if (strcmp(key, "wing_cl0_camber") == 0){ vehcle.wing.cl0_camber = val; }
    else if (strcmp(key, "wing_k_w") == 0)       { vehcle.wing.k_w = val; }
    else if (strcmp(key, "wing_r") == 0)         { vehcle.wing.r = val; }
    // Tail
    else if (strcmp(key, "tail_a_t") == 0)       { vehcle.tail.a_t = val; }
    else if (strcmp(key, "tail_eta_t_0") == 0)   { vehcle.tail.eta_t_0 = val; }
    else if (strcmp(key, "tail_eta_t_1") == 0)   { vehcle.tail.eta_t_1 = val; }
    else if (strcmp(key, "tail_eps0") == 0)      { vehcle.tail.eps0 = val; }
    else if (strcmp(key, "tail_deps_pos") == 0)  { vehcle.tail.deps_pos = val; }
    else if (strcmp(key, "tail_deps_neg") == 0)  { vehcle.tail.deps_neg = val; }
    else if (strcmp(key, "tail_k_ht") == 0)      { vehcle.tail.k_ht = val; }
    else if (strcmp(key, "tail_CD_ht0") == 0)    { vehcle.tail.CD_ht0 = val; }
    else if (strcmp(key, "tail_S_ht_S") == 0)    { vehcle.tail.S_ht_S = val; }
    else if (strcmp(key, "tail_V_H") == 0)       { vehcle.tail.V_H = val; }
    else if (strcmp(key, "tail_AR_ht") == 0)     { vehcle.tail.AR_ht = val; }
    // Fuselage
    else if (strcmp(key, "fuse_CD_a2") == 0)     { vehcle.fuse.CD_a2 = val; }
    else if (strcmp(key, "fuse_CD_b2") == 0)     { vehcle.fuse.CD_b2 = val; }
    else if (strcmp(key, "fuse_CD_a2_Cmyu") == 0){ vehcle.fuse.CD_a2_Cmyu = val; }
    else if (strcmp(key, "fuse_CD0") == 0)       { vehcle.fuse.CD0 = val; }
    else if (strcmp(key, "fuse_CDp_cu") == 0)    { vehcle.fuse.CDp_cu = val; }
    else if (strcmp(key, "fuse_CDq_cu") == 0)    { vehcle.fuse.CDq_cu = val; }
    else if (strcmp(key, "fuse_CDr_cu") == 0)    { vehcle.fuse.CDr_cu = val; }
    // Controls
    else if (strcmp(key, "controls_tau_f") == 0) { vehcle.controls.tau_f = val; }
    else if (strcmp(key, "controls_tau_a") == 0) { vehcle.controls.tau_a = val; }
    else if (strcmp(key, "controls_Kb_f") == 0)  { vehcle.controls.Kb_f = val; }
    else if (strcmp(key, "controls_Kb_a") == 0)  { vehcle.controls.Kb_a = val; }
    else if (strcmp(key, "controls_tau_e") == 0) { vehcle.controls.tau_e = val; }
    else if (strcmp(key, "controls_CD_df2") == 0){ vehcle.controls.CD_df2 = val; }
    else if (strcmp(key, "controls_CD_da2") == 0){ vehcle.controls.CD_da2 = val; }
    else if (strcmp(key, "controls_CD_da") == 0) { vehcle.controls.CD_da = val; }
    else if (strcmp(key, "controls_CD_de2") == 0){ vehcle.controls.CD_de2 = val; }
    else if (strcmp(key, "controls_CD_de") == 0) { vehcle.controls.CD_de = val; }
    else if (strcmp(key, "controls_CD_dr2") == 0){ vehcle.controls.CD_dr2 = val; }
    // Stall (drag blend)
    else if (strcmp(key, "stall_K_flat") == 0)   { vehcle.stall.K_flat = val; }
    else if (strcmp(key, "stall_a0_const") == 0) { vehcle.stall.a0_const = val; }
    else if (strcmp(key, "stall_a0_Cmyu") == 0)  { vehcle.stall.a0_Cmyu = val; }
    else if (strcmp(key, "stall_k") == 0)        { vehcle.stall.k = val; }
    // Post-stall lift blend
    else if (strcmp(key, "cl_stall_wM0") == 0)   { vehcle.cl_stall.wM0 = val; }
    else if (strcmp(key, "cl_stall_wM1") == 0)   { vehcle.cl_stall.wM1 = val; }
    else if (strcmp(key, "cl_stall_wa00") == 0)  { vehcle.cl_stall.wa00 = val; }
    else if (strcmp(key, "cl_stall_wa0mu") == 0) { vehcle.cl_stall.wa0mu = val; }
    else if (strcmp(key, "cl_stall_wa0f") == 0)  { vehcle.cl_stall.wa0f = val; }
    else if (strcmp(key, "cl_stall_wkflat") == 0){ vehcle.cl_stall.wkflat = val; }
    else if (strcmp(key, "cl_stall_rM") == 0)    { vehcle.cl_stall.rM = val; }
    else if (strcmp(key, "cl_stall_ra0") == 0)   { vehcle.cl_stall.ra0 = val; }
    else if (strcmp(key, "cl_stall_rkcu") == 0)  { vehcle.cl_stall.rkcu = val; }
    else if (strcmp(key, "cl_stall_rkflat") == 0){ vehcle.cl_stall.rkflat = val; }
    // Lateral side force (CY)
    else if (strcmp(key, "lateral_theta0") == 0)  { vehcle.lateral.theta0 = val; }
    else if (strcmp(key, "lateral_theta_b") == 0) { vehcle.lateral.theta_b = val; }
    else if (strcmp(key, "lateral_theta_aL") == 0){ vehcle.lateral.theta_aL = val; }
    else if (strcmp(key, "lateral_theta_aR") == 0){ vehcle.lateral.theta_aR = val; }
    else if (strcmp(key, "lateral_theta_r") == 0) { vehcle.lateral.theta_r = val; }
    else if (strcmp(key, "lateral_theta_bcu") == 0){ vehcle.lateral.theta_bcu = val; }
    else if (strcmp(key, "lateral_beta0") == 0)   { vehcle.lateral.beta0 = val; }
    else if (strcmp(key, "lateral_kv") == 0)      { vehcle.lateral.kv = val; }
    else if (strcmp(key, "lateral_kps_r") == 0)   { vehcle.lateral.kps_r = val; }
    else if (strcmp(key, "lateral_M") == 0)       { vehcle.lateral.M = val; }
    else if (strcmp(key, "lateral_CYp") == 0)     { vehcle.lateral.CYp = val; }
    else if (strcmp(key, "lateral_CYr") == 0)     { vehcle.lateral.CYr = val; }
    else if (strcmp(key, "lateral_CYp_cu") == 0)  { vehcle.lateral.CYp_cu = val; }
    else if (strcmp(key, "lateral_CYr_cu") == 0)  { vehcle.lateral.CYr_cu = val; }
    else if (strcmp(key, "lateral_CYp2_cu") == 0) { vehcle.lateral.CYp2_cu = val; }
    else if (strcmp(key, "lateral_CYr2_cu") == 0) { vehcle.lateral.CYr2_cu = val; }
    // Rolling moment (Cl)
    else if (strcmp(key, "roll_theta0") == 0)     { vehcle.roll.theta0 = val; }
    else if (strcmp(key, "roll_theta_aL") == 0)   { vehcle.roll.theta_aL = val; }
    else if (strcmp(key, "roll_theta_aR") == 0)   { vehcle.roll.theta_aR = val; }
    else if (strcmp(key, "roll_theta_aLcu") == 0) { vehcle.roll.theta_aLcu = val; }
    else if (strcmp(key, "roll_theta_aRcu") == 0) { vehcle.roll.theta_aRcu = val; }
    else if (strcmp(key, "roll_theta_b") == 0)    { vehcle.roll.theta_b = val; }
    else if (strcmp(key, "roll_theta_b_cu") == 0) { vehcle.roll.theta_b_cu = val; }
    else if (strcmp(key, "roll_theta_r") == 0)    { vehcle.roll.theta_r = val; }
    else if (strcmp(key, "roll_Clp") == 0)        { vehcle.roll.Clp = val; }
    else if (strcmp(key, "roll_Clr") == 0)        { vehcle.roll.Clr = val; }
    else if (strcmp(key, "roll_Clp_cu") == 0)     { vehcle.roll.Clp_cu = val; }
    else if (strcmp(key, "roll_Clr_cu") == 0)     { vehcle.roll.Clr_cu = val; }
    // Yawing moment (Cn)
    else if (strcmp(key, "yaw_theta0") == 0)      { vehcle.yaw.theta0 = val; }
    else if (strcmp(key, "yaw_theta_b") == 0)     { vehcle.yaw.theta_b = val; }
    else if (strcmp(key, "yaw_theta_aL") == 0)    { vehcle.yaw.theta_aL = val; }
    else if (strcmp(key, "yaw_theta_aR") == 0)    { vehcle.yaw.theta_aR = val; }
    else if (strcmp(key, "yaw_theta_r") == 0)     { vehcle.yaw.theta_r = val; }
    else if (strcmp(key, "yaw_theta_bcu") == 0)   { vehcle.yaw.theta_bcu = val; }
    else if (strcmp(key, "yaw_theta_aLcu") == 0)  { vehcle.yaw.theta_aLcu = val; }
    else if (strcmp(key, "yaw_theta_aRcu") == 0)  { vehcle.yaw.theta_aRcu = val; }
    else if (strcmp(key, "yaw_Cnp") == 0)         { vehcle.yaw.Cnp = val; }
    else if (strcmp(key, "yaw_Cnp_cu") == 0)      { vehcle.yaw.Cnp_cu = val; }
    else if (strcmp(key, "yaw_Cnr") == 0)         { vehcle.yaw.Cnr = val; }
    else if (strcmp(key, "yaw_Cnr_cu") == 0)      { vehcle.yaw.Cnr_cu = val; }
    else if (strcmp(key, "yaw_beta0") == 0)       { vehcle.yaw.beta0 = val; }
    else if (strcmp(key, "yaw_kcu") == 0)         { vehcle.yaw.kcu = val; }
    else if (strcmp(key, "yaw_kv") == 0)          { vehcle.yaw.kv = val; }
    else if (strcmp(key, "yaw_kps_r") == 0)       { vehcle.yaw.kps_r = val; }
    else if (strcmp(key, "yaw_M") == 0)           { vehcle.yaw.M = val; }
    // Propulsion (thrust / blowing coeff fits — used in rotor dynamics)
    else if (strcmp(key, "prop_cmyu_J2") == 0)    { vehcle.prop.cmyu_J2 = val; }
    else if (strcmp(key, "prop_cmyu_0") == 0)     { vehcle.prop.cmyu_0 = val; }
    else if (strcmp(key, "prop_CT_1") == 0)       { vehcle.prop.CT_1 = val; }
    else if (strcmp(key, "prop_CT_J") == 0)       { vehcle.prop.CT_J = val; }
    else if (strcmp(key, "prop_CT_JM") == 0)      { vehcle.prop.CT_JM = val; }
    else {
        printf("[MonteCarlo] WARNING: unknown param '%s' — skipped\n", key);
    }
}

// ============================================================
void v_apply_monte_carlo_overrides()
{
    // Determine override file path: env var first, then default
    const char* override_path = getenv("LAT_MC_OVERRIDE_FILE");
    if (override_path == nullptr || override_path[0] == '\0') {
        override_path = MC_OVERRIDE_FILE_DEFAULT;
    }

    FILE* fp = fopen(override_path, "r");
    if (fp == nullptr) {
        // No override file → nominal run.  Completely silent.
        return;
    }

    printf("[MonteCarlo] Reading overrides from: %s\n", override_path);

    s_cg_x_overridden = false;
    char line[256];
    int count = 0;

    while (fgets(line, sizeof(line), fp) != nullptr) {
        // Skip comments and blank lines
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }

        // Remove trailing newline
        char* nl = strchr(line, '\n');
        if (nl) *nl = '\0';
        nl = strchr(line, '\r');
        if (nl) *nl = '\0';

        // Split on '='
        char* eq = strchr(line, '=');
        if (eq == nullptr) {
            continue;  // malformed line
        }

        *eq = '\0';
        const char* key = line;
        float val = (float)atof(eq + 1);

        apply_override(key, val);
        printf("[MonteCarlo]   %s = %.6f\n", key, (double)val);
        count++;
    }

    fclose(fp);

    // Recompute derived quantities that depend on overridden params.
    //
    // Landing-gear geometry: only recompute when the DIMENSIONAL cg_x was
    // overridden (the Equinox path). uSTOL perturbs cg.x_cg_c (non-dimensional)
    // and sets MLG/FLG explicitly in v_plane_param_define_ustol_v1(), so this
    // legacy nose-reference formula must NOT clobber it on a uSTOL run.
    if (s_cg_x_overridden) {
        vehcle.MLG_x = fabsf((1319.2f / 1000.0f) - vehcle.cg_x);
        vehcle.MLG_z = fabsf(319.6f / 1000.0f + vehcle.cg_z);
        vehcle.FLG_x = fabsf((339.2f / 1000.0f) - vehcle.cg_x);
        vehcle.FLG_z = vehcle.MLG_z;
    }

    // Aspect ratio is derived from span/area at define time; recompute so an
    // s or b perturbation actually propagates into the induced-drag terms.
    vehcle.AR = (vehcle.b * vehcle.b) / vehcle.s;

    // Static thrust coefficient is derived from max_thrust and rho at define
    // time; recompute so a max_thrust or rho perturbation changes the thrust.
    for (int i = 0; i < s_motor_manager.num_motors; i++) {
        s_motor[i].CT_static = s_motor[i].max_thrust /
            (vehcle.rho * powf(s_motor[i].dia_prop, 4) *
             powf(s_motor[i].rpm_max / 60.0f, 2));
    }

    printf("[MonteCarlo] Applied %d overrides.\n", count);
}
