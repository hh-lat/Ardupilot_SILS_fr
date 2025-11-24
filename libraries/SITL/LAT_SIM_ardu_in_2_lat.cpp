
#include "LAT_SIM_ardu_in_2_lat.h"
#include "LAT_SIM_servo_dynamics.h"
#include "LAT_SIM_rotor_dynamics.h"
#include "LAT_SIM_Runner.h"



// Sign convention Ardupilot follows in non revered ticked state:
// Elevator: 1900 means elevator down in both manual and stabalise mode
// Rudder: 1900 means rudder to the right-most in both manual and stabalise mode
// Aileron_common: 1900 means right aileron up and left aileron down in both manual and stabalise mode
// Throttle: 1900 means max throttle in both Manual and stabilise mode
// Flap: 1900 means Flap down

//Note: If reverse is ticked in Missionplanner against any channel, then above convention reverses for that channel, so accordingly also change mapping in servo_dynamics of FDM


// Sign convention LAT FDM follows: Aerospace Standard Sign Convention
// Elevator: Elevator down is +ve angle, hence for conventional plane cm_delta_e should be -ve in FDM,    example:  Cm_delta_e = -0.5 for Aerosonde
// Aileron_commom: Aileron up is +ve angle, hence for conventional plane cl_delta_a should be +ve in FDM, example:  Cl_delta_a =  0.08 for Aerosonde
// Rudder_common: Rudder left is +ve angle, hence for conventional plane cn_delta_r should be -ve, example in FDM:  Cn_delta_r = -0.032 for Aerosonde
// Aileron right: Right Aileron up is +ve angle
// Aileron left: Left Aileron down is +ve angle

// Aileron common to left right Ambiguity:
// if using single(combined coffecient) aileron(aileron_common) philosphy, then just convert pwm to angle and multiply by the combined coffecient, most conventional aircrafts and FDMs uses combined coeffecient philosphy
// if using left-right aileron in FDM , just ensure you have dedicated coffecients for left aileron and seperate dedicated coefficient for right aileron
// once you have dedicated coeffecient just convert aileron channel to angle and diretly use it, in an ideal world the dedicated coffecient's are 1/2 of combined coeffeceint with opposite signs
void  v_ardu_input_to_lat_input(struct sitl_input input)
{

    switch(vehcle.plane_model)
    {
        case PLANE_ARDU_DEFAULT:
        {
            s_servo[AILERON_LEFT].pwm_in    = input.servos[9]; // aileron left
            s_servo[AILERON_RIGHT].pwm_in   = input.servos[10]; // aileron right

            s_servo[ELEVATOR_COMMON].pwm_in = input.servos[12]; //elevator common
            s_servo[RUDDER_COMMON].pwm_in   = input.servos[5]; //Rudder common
            s_servo[FLAP].pwm_in            = input.servos[11]; //rudder common
            s_servo[NOSE_LG_SERVO].pwm_in   = input.servos[8];

            s_motor[0].pwm_in = input.servos[0]; // motor 1
            s_motor[1].pwm_in = input.servos[1]; // motor 2
            s_motor[2].pwm_in = input.servos[2]; // motor 3
            s_motor[3].pwm_in = input.servos[3]; // motor 4
            s_motor[4].pwm_in = input.servos[3]; // motor 5
            s_motor[5].pwm_in = input.servos[2]; // motor 6
            s_motor[6].pwm_in = input.servos[1]; // motor 7
            s_motor[7].pwm_in = input.servos[0]; // motor 8            
            break;
        }

        case PLANE_EQX:
        {
            s_servo[AILERON_LEFT].pwm_in    = input.servos[0]; // aileron left
            s_servo[AILERON_RIGHT].pwm_in   = input.servos[1]; // aileron right

            s_servo[ELEVATOR_COMMON].pwm_in = input.servos[2]; //elevator common
            s_servo[RUDDER_COMMON].pwm_in   = input.servos[2]; //elevator common
            s_servo[FLAP].pwm_in            = input.servos[3]; //rudder common
            s_servo[NOSE_LG_SERVO].pwm_in   = input.servos[4];

            s_motor[0].pwm_in = input.servos[0]; // motor 1
            s_motor[1].pwm_in = input.servos[1]; // motor 2
            s_motor[2].pwm_in = input.servos[2]; // motor 3
            s_motor[3].pwm_in = input.servos[3]; // motor 4
            s_motor[4].pwm_in = input.servos[3]; // motor 5
            s_motor[5].pwm_in = input.servos[2]; // motor 6
            s_motor[6].pwm_in = input.servos[1]; // motor 7
            s_motor[7].pwm_in = input.servos[0]; // motor 8  
            break;
        }


    }
}





