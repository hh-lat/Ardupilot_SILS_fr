
#include "LAT_SIM_ardu_in_2_lat.h"
#include "LAT_SIM_servo_dynamics.h"
#include "LAT_SIM_rotor_dynamics.h"
#include "LAT_SIM_Runner.h"

#define flap_pwm_hardcoded 1600   // 20 deg flap: FLAP range 0-32 deg over PWM 1100-1900, so 1100+(20/32)*800 = 1600

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

            s_servo[AILERON_COMMON].pwm_in  = s_servo[AILERON_RIGHT].pwm_in; // aileron common

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
            s_servo[AILERON_LEFT].pwm_in    = input.servos[9]; // aileron left
            s_servo[AILERON_RIGHT].pwm_in   = input.servos[10]; // aileron right

            s_servo[AILERON_COMMON].pwm_in  = s_servo[AILERON_RIGHT].pwm_in; // aileron common

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

        case PLANE_EQX_V1_NEW_MODEL:
        {
            s_servo[AILERON_LEFT].pwm_in    = input.servos[9]; // aileron left
            s_servo[AILERON_RIGHT].pwm_in   = input.servos[10]; // aileron right

            s_servo[AILERON_COMMON].pwm_in  = s_servo[AILERON_RIGHT].pwm_in; // aileron common

            s_servo[ELEVATOR_COMMON].pwm_in = input.servos[12]; //elevator common
            s_servo[RUDDER_COMMON].pwm_in   = input.servos[5]; //Rudder common
            s_servo[FLAP].pwm_in            = input.servos[11]; //flap_auto (SERVO12, fn=3)
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

        case PLANE_USTOL_V1:
        {
            // ---- Output-channel map (set SERVOn_FUNCTION in the uSTOL .parm to match) ----
            //   SERVO1..9 -> 18 EDFs, port wingtip to starboard wingtip: ch1={1} ch2={2,3}
            //   ch3={4,5} ch4={6,7} ch5={8,9,10,11} ch6={12,13} ch7={14,15} ch8={16,17} ch9={18}
            //   SERVO10 -> aileron   SERVO11 -> elevator   SERVO12 -> rudder
            //   SERVO13 -> flap (rotation; drives aero delta_f)
            //   SERVO14 -> flap (linear/Fowler extension; NOT used aerodynamically here)

            // Control surfaces. One aileron channel drives both ailerons; AILERON_LEFT
            // has reversed travel in the param block, so the same PWM -> opposite L/R.
            s_servo[AILERON_COMMON].pwm_in  = input.servos[9];
            s_servo[AILERON_LEFT].pwm_in    = input.servos[9];
            s_servo[AILERON_RIGHT].pwm_in   = input.servos[9];
            s_servo[ELEVATOR_COMMON].pwm_in = input.servos[10];
            s_servo[RUDDER_COMMON].pwm_in   = input.servos[11];
            s_servo[FLAP].pwm_in            = flap_pwm_hardcoded;  // HARD-CODED 20 deg flap, all phases (ignores ArduPlane flap output)
            s_servo[NOSE_LG_SERVO].pwm_in   = 1500;               // not modelled; neutral

            // 18 EDFs on 9 throttle channels - confirmed physical grouping, port wingtip
            // to starboard wingtip: ch1={1} ch2={2,3} ch3={4,5} ch4={6,7}
            // ch5={8,9,10,11} (straddles fuselage) ch6={12,13} ch7={14,15} ch8={16,17} ch9={18}.
            // s_motor[i] = EDF (i+1); matches the y-station remap in v_plane_param_define_ustol_v1().
            static const uint8_t ch_first[9] = {0, 1, 3, 5, 7, 11, 13, 15, 17};
            static const uint8_t ch_count[9] = {1, 2, 2, 2, 4, 2, 2, 2, 1};
            for (int p = 0; p < 9; p++)
            {
                for (int j = 0; j < ch_count[p]; j++)
                {
                    s_motor[ch_first[p] + j].pwm_in = input.servos[p];
                }
            }
            break;
        }
    }
}





