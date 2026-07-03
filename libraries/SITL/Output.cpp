/*
 * Output.cpp
 * ----------
 * Logs simulation state data to a CSV text file for post-run analysis.
 * Output directory: $LAT_SIM_LOG_DIR or <workspace>/Logs_Simulations/
 */

#include "Output.h"
#include "LAT_SIM_Runner.h"
#include "LAT_SIM_servo_dynamics.h"
#include "LAT_SIM_rotor_dynamics.h"
#include <AP_Vehicle/AP_Vehicle.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <libgen.h>
#include <limits.h>

static FILE* log_fp = nullptr;
static const char LOG_DIR_DEFAULT[] = "/home/sushanthvenkata/Projects/Ardupilot_SILS_v2/Ardupilot_SILS/ustol_sims/logs/";

static const char* get_log_dir()
{
    // Allow override via environment variable for portability
    const char* env = getenv("LAT_SIM_LOG_DIR");
    if (env && env[0] != '\0') {
        return env;
    }
    return LOG_DIR_DEFAULT;
}

static void ensure_dir_exists(const char* dir)
{
    // Recursively create the directory path (mkdir -p semantics), so a missing
    // parent (e.g. ustol_sims/ on a fresh checkout) can't silently fail and
    // leave the run without a log, as the old single-level mkdir did.
    char tmp[512];
    snprintf(tmp, sizeof(tmp), "%s", dir);
    size_t len = strlen(tmp);
    if (len > 0 && tmp[len - 1] == '/') {
        tmp[len - 1] = '\0';
    }
    for (char* p = tmp + 1; *p != '\0'; p++) {
        if (*p == '/') {
            *p = '\0';
            mkdir(tmp, 0755);
            *p = '/';
        }
    }
    mkdir(tmp, 0755);
}

static void build_filename(char* buf, size_t len)
{
    const char* log_dir = get_log_dir();
    ensure_dir_exists(log_dir);

    time_t now = time(nullptr);
    struct tm* tm_info = localtime(&now);
    int pid = (int)getpid();

    snprintf(buf, len,
             "%ssim_output_%04d%02d%02d_%02d%02d%02d_pid%d.csv",
             log_dir,
             tm_info->tm_year + 1900,
             tm_info->tm_mon + 1,
             tm_info->tm_mday,
             tm_info->tm_hour,
             tm_info->tm_min,
             tm_info->tm_sec,
             pid);
}

void v_output_log_init()
{
    if (log_fp != nullptr) {
        return;
    }

    char filepath[512];
    build_filename(filepath, sizeof(filepath));

    log_fp = fopen(filepath, "w");
    if (log_fp == nullptr) {
        printf("[Output] ERROR: could not open log file: %s\n", filepath);
        return;
    }

    printf("[Output] Logging to: %s\n", filepath);

    // CSV header
    fprintf(log_fp,
        "Time_s,"
        "plane_moving_state,"
        "TAS_mps,"
        "alt_agl_m,"
        "MLG_NR,"
        "FLG_NR,"
        "lat,"
        "lon,"
        "alt_msl,"
        "phi,"
        "theta,"
        "psi,"
        "p,"
        "q,"
        "r,"
        "V_b_tas_0,"
        "V_b_tas_1,"
        "V_b_tas_2,"
        "V_ned_gnd_0,"
        "V_ned_gnd_1,"
        "V_ned_gnd_2,"
        "pos_ned_0,"
        "pos_ned_1,"
        "pos_ned_2,"
        "p_dot,"
        "q_dot,"
        "r_dot,"
        "Accel_b_0,"
        "Accel_b_1,"
        "Accel_b_2,"
        "ArduPlane_Mode,"
        "delta_e,"
        "delta_aL,"
        "delta_aR,"
        "delta_r,"
        "delta_f,"
        "delta_a,"
        "delta_e_cmd,"
        "delta_aL_cmd,"
        "delta_aR_cmd,"
        "delta_r_cmd,"
        "slew_e_dps,"
        "slew_aL_dps,"
        "slew_aR_dps,"
        "slew_r_dps,"
        "pwm_ailL,"
        "pwm_ailR,"
        "pwm_elev,"
        "pwm_rud,"
        "pwm_flap,"
        "pwm_nlg,"
        "pwm_mot0,"
        "mot0_thr_cmd,"
        "mot1_thr_cmd,"
        "total_rotor_force,"
        "Lift_Coeff,"
        "Moment_Coeff,"
        "Drag_Coeff,"
        "Lift_N,"
        "Drag_N,"
        "Side_N,"
        "J,"
        "Cmu,"
        "W_clw,W_clr,W_cd,W_cy,W_cn,"
        "x_cp_ac,x_cp_w,x_ac_w"
        "\n");

    fflush(log_fp);
}

void v_output_log_write(float t)
{
    if (log_fp == nullptr) {
        return;
    }

    // Get current ArduPlane mode number
    int ardu_mode = -1;
    AP_Vehicle *veh = AP_Vehicle::get_singleton();
    if (veh != nullptr) {
        ardu_mode = (int)veh->get_mode();
    }

    fprintf(log_fp,
        "%.5f,%d,%.5f,%.5f,"
        "%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%d,"
        "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,%.5f,%.1f,%.1f,%.1f,%.1f,"
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
        "%.1f,"
        "%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
        t,
        (int)vehcle.plane_moving_state,
        vehcle.tas,
        vehcle.alt_agl,
        vehcle.MLG_NR,
        vehcle.FLG_NR,
        vehcle.lat,
        vehcle.lon,
        vehcle.alt_msl,
        vehcle.phi,
        vehcle.theta,
        vehcle.psi,
        vehcle.p,
        vehcle.q,
        vehcle.r,
        vehcle.V_b_tas[0],
        vehcle.V_b_tas[1],
        vehcle.V_b_tas[2],
        vehcle.V_ned_gnd[0],
        vehcle.V_ned_gnd[1],
        vehcle.V_ned_gnd[2],
        vehcle.pos_ned[0],
        vehcle.pos_ned[1],
        vehcle.pos_ned[2],
        vehcle.p_dot,
        vehcle.q_dot,
        vehcle.r_dot,
        vehcle.Accel_b[0],
        vehcle.Accel_b[1],
        vehcle.Accel_b[2],
        ardu_mode,
        vehcle.delta_e,
        vehcle.delta_aL,
        vehcle.delta_aR,
        vehcle.delta_r,
        vehcle.delta_f,
        vehcle.delta_a,
        s_servo[ELEVATOR_COMMON].angle_cmd,
        s_servo[AILERON_LEFT].angle_cmd,
        s_servo[AILERON_RIGHT].angle_cmd,
        s_servo[RUDDER_COMMON].angle_cmd,
        s_servo[ELEVATOR_COMMON].slew_used*R2D,
        s_servo[AILERON_LEFT].slew_used*R2D,
        s_servo[AILERON_RIGHT].slew_used*R2D,
        s_servo[RUDDER_COMMON].slew_used*R2D,
        s_servo[AILERON_LEFT].pwm_in,
        s_servo[AILERON_RIGHT].pwm_in,
        s_servo[ELEVATOR_COMMON].pwm_in,
        s_servo[RUDDER_COMMON].pwm_in,
        s_servo[FLAP].pwm_in,
        s_servo[NOSE_LG_SERVO].pwm_in,
        s_motor[0].pwm_in,
        s_motor[0].throttle_cmd,
        s_motor[1].throttle_cmd,
        vehcle.all_rotors_force[0],
        vehcle.CL,
        vehcle.Cm,
        vehcle.CD,
        vehcle.all_lift_force,
        vehcle.all_drag_force,
        vehcle.all_side_force,
        s_motor[0].J,
        s_motor[0].Cmu,
        vehcle.W_clw, vehcle.W_clr, vehcle.W_cd, vehcle.W_cy, vehcle.W_cn,
        vehcle.x_cp_ac, vehcle.x_cp_w, vehcle.x_ac_w
        );

    fflush(log_fp);
}

void v_output_log_close()
{
    if (log_fp != nullptr) {
        fflush(log_fp);
        fclose(log_fp);
        log_fp = nullptr;
        printf("[Output] Log file closed.\n");
    }
}
