
#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class LAT {
public:
    static const struct AP_Param::GroupInfo var_info[];
    void lat_update_motor_state();
    void reset_bad_flags();
    static const uint8_t N_MOTORS = 8;

   

        // getter to expose bad flags
    bool is_motor_bad(uint8_t idx) const {
        return (idx < N_MOTORS) ? motor_bad[idx] : false;
    }

        // Params
    AP_Int8   det_mode;     
    AP_Float  v_thresh;     
    AP_Float  i_thresh;     
    AP_Float  rpm_thresh;     
    AP_Float  temp_thresh;     
    AP_Int16   consec_req;   

    //    LAT() {
    //     // must link params to EEPROM system
    //     AP_Param::setup_object_defaults(this, var_info);
    // }
    


private:

    // Runtime state
    bool motor_bad[N_MOTORS] = {false};
    uint8_t bad_count[N_MOTORS] = {0};

    // 🔹 helper
    float compute_avg(float *arr, bool *valid, uint8_t N, bool *motor_bd);
};

//extern LAT lat1; // global instance of LAT for use in other files