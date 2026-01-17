#include "lat.h"
#include "AP_ESC_Telem/AP_ESC_Telem.h"
#include <AP_Param/AP_Param.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_HAL/AP_HAL.h>
//#include "Plane.h"
//#include "ArduPlane/Parameters.h" 

// bitmask detection modes
#define DET_MODE_VOLTAGE    0x01
#define DET_MODE_CURRENT    0x02
#define DET_MODE_RPM        0x04
#define DET_MODE_TEMP       0x08

// ---------------- PARAMETER TABLE ----------------
const AP_Param::GroupInfo LAT::var_info[] = {
    // @Param: DET_MODE
    // @DisplayName: Detection Mode
    // @Description: Bitmask to enable motor fault detection sources
    // @Values: 0:Disable,1:Voltage,2:Current,4:RPM,8:Temperature,15:All
    // @User: Advanced
    AP_GROUPINFO("DET_MODE", 0, LAT, det_mode, 0), // default = all enabled (1111b = 15)

    // @Param: V_THR
    // @DisplayName: Voltage threshold
    // @Description: Relative voltage drop threshold to mark motor bad
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("V_THR", 1, LAT, v_thresh, 0.15f),

    // @Param: I_THR
    // @DisplayName: Current threshold
    // @Description: Relative current rise/drop threshold
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("I_THR", 2, LAT, i_thresh, 0.20f),

    // @Param: R_THR
    // @DisplayName: RPM threshold
    // @Description: Relative RPM drop threshold
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("R_THR", 3, LAT, rpm_thresh, 0.25f),

    // @Param: T_THR
    // @DisplayName: Temperature threshold
    // @Description: Relative temperature deviation threshold
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("T_THR", 4, LAT, temp_thresh, 0.30f),

    // @Param: CONSEC
    // @DisplayName: Consecutive bad iterations
    // @Description: Number of consecutive bad detections required before flagging motor as bad
    // @Range: 0 32767
    // @User: Advanced
    AP_GROUPINFO("CONSEC", 5, LAT, consec_req, 3),

    AP_GROUPEND
};


// ---------------- CONSTRUCTOR ----------------
// LAT::LAT()
// {
//     // Bind parameters to AP_Param system
//     AP_Param::setup_object_defaults(this, var_info);

//     // initialise runtime state
//     memset(motor_bad, 0, sizeof(motor_bad));
// }

// ---------------- MOTOR STATE UPDATE ----------------
void LAT::lat_update_motor_state()
{
   // int mode = det_mode.get();

     int mode = 0;//plane.g2.det_mode.get();
     //int mode = _params.det_mode.get();

    // do not check if disarmed or when no detection mode is set, reset motor flags
    if ((!AP::arming().is_armed()) || (mode == 0)) {
        reset_bad_flags();
        return;
    }

    AP_ESC_Telem &telem = AP::esc_telem();

    //const uint8_t N_MOTORS = 8;
    float voltages[N_MOTORS] = {0}, currents[N_MOTORS] = {0}, rpms[N_MOTORS] = {0}, temps[N_MOTORS] = {0};
    bool valid_voltage[N_MOTORS] = {false}, valid_current[N_MOTORS] = {false}, valid_rpm[N_MOTORS] = {false}, valid_temp[N_MOTORS] = {false};

    // --- collect data ---
    for (uint8_t i = 0; i < N_MOTORS; i++) 
    {
        if (telem.get_voltage(i, voltages[i]))
        {
            valid_voltage[i] = true;
        } 
        if (telem.get_current(i, currents[i])) 
        {
            valid_current[i] = true;
        }

        if (telem.get_rpm(i, rpms[i]))  
        {
            valid_rpm[i] = true;
        }

        int16_t temp_cdeg;
        if (telem.get_temperature(i, temp_cdeg)) {
            temps[i] = temp_cdeg * 0.01f;
            valid_temp[i] = true;
        }
    }

    float v_avg = compute_avg(voltages, valid_voltage, N_MOTORS,motor_bad);
    float i_avg = compute_avg(currents, valid_current, N_MOTORS,motor_bad);
    float rpm_avg = compute_avg(rpms, valid_rpm, N_MOTORS,motor_bad);
    float temp_avg = compute_avg(temps, valid_temp, N_MOTORS,motor_bad);

    if ((v_avg < 15) || (i_avg < 0.1) || (rpm_avg <10) || (temp_avg < -20)) 
    {
        // If any average is zero, we cannot proceed with detection
        return;
    }



    // --- detection ---
    for (uint8_t i = 0; i < N_MOTORS; i++) 
    {
        if (motor_bad[i]) 
        {
            continue;
        }

        bool bad=false;

        if ((det_mode & DET_MODE_VOLTAGE) && valid_voltage[i]) {
            if (fabsf(voltages[i] - v_avg) / v_avg > v_thresh) {
                bad = true;
            }
        }

        if ((det_mode & DET_MODE_CURRENT) && valid_current[i]) {
            if (fabsf(currents[i] - i_avg) / i_avg > i_thresh) {
                bad = true;
            }
        }

        if ((det_mode & DET_MODE_RPM) && valid_rpm[i]) {
            if (fabsf(rpms[i] - rpm_avg) / rpm_avg > rpm_thresh) {
                bad = true;
            }
        }

        if ((det_mode & DET_MODE_TEMP) && valid_temp[i]) {
            if (fabsf(temps[i] - temp_avg) / temp_avg > temp_thresh) {
                bad = true;
            }
        }

        if (bad==true) 
        {
            bad_count[i]++;
            if (bad_count[i] >= consec_req) 
            {
                motor_bad[i] = true;
                motor_bad[N_MOTORS-1-i] = true; //make counter motor also bad
            }
        } 
        else 
        {
            bad_count[i] = 0;
        }
    }
}

// ---------------- RESET ----------------
void LAT::reset_bad_flags()
{
    for (uint8_t i=0; i<8; i++) {
        motor_bad[i] = false;
        bad_count[i] = 0;
    }
}


float LAT::compute_avg(float *arr, bool *valid, uint8_t N, bool *motor_bd)
{
    float sum = 0;
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < N; i++) {
        if (valid[i]&& (motor_bd[i] == false)) {
            sum += arr[i];
            cnt++;
        }
    }
    return (cnt > 0) ? sum / cnt : 0.0f;
}