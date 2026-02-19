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

// ---- Map a key string to the correct vehcle field ----
static void apply_override(const char* key, float val)
{
    // CG
    if      (strcmp(key, "cg_x") == 0)        { vehcle.cg_x = val; }
    else if (strcmp(key, "cg_z") == 0)        { vehcle.cg_z = val; }
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

    // Recompute derived quantities that depend on overridden params
    vehcle.MLG_x = fabsf((1319.2f / 1000.0f) - vehcle.cg_x);
    vehcle.MLG_z = fabsf(319.6f / 1000.0f + vehcle.cg_z);
    vehcle.FLG_x = fabsf((339.2f / 1000.0f) - vehcle.cg_x);
    vehcle.FLG_z = vehcle.MLG_z;
 
    printf("[MonteCarlo] Applied %d overrides.\n", count);
}
