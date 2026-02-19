/*
 * Output.cpp
 * ----------
 * Logs simulation state data to a CSV text file for post-run analysis.
 * Output directory: /home/lat_avionics/Ardupilot_SITL_LATEST/Ardupilot_SILS/Logs_Simulations/
 */

#include "Output.h"
#include "LAT_SIM_Runner.h"
#include "LAT_SIM_servo_dynamics.h"
#include "LAT_SIM_rotor_dynamics.h"
#include <AP_Vehicle/AP_Vehicle.h>
#include <time.h>
#include <string.h>
#include <unistd.h>

static FILE* log_fp = nullptr;
static const char LOG_DIR[] = "/home/lat_avionics/Ardupilot_SITL_LATEST/Ardupilot_SILS/Logs_Simulations/";

static void build_filename(char* buf, size_t len)
{
    time_t now = time(nullptr);
    struct tm* tm_info = localtime(&now);
    int pid = (int)getpid();

    snprintf(buf, len,
             "%ssim_output_%04d%02d%02d_%02d%02d%02d_pid%d.csv",
             LOG_DIR,
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
        "Moment_Coeff"
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
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
        "%.1f,"
        "%.5f,%.5f,"
        "%.5f,%.5f,%.5f\n",
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
        vehcle.Cm
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
