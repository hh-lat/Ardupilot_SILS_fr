
#include "LAT_SIM_ardu_in_2_lat.h"
#include "LAT_SIM_Actuator_dynamics.h"
#include "LAT_SIM_rotor_dynamics.h"

void  v_ardu_input_to_lat_input(struct sitl_input &input)
{
    v_map_ardu_in_for_equinox(input);
}



void v_map_ardu_in_for_equinox(struct sitl_input &input)
{
    actuator[AILERON].pwm  = input.servos[0];
    actuator[ELEVATOR].pwm = input.servos[1];
    actuator[RUDDER].pwm   = input.servos[2];
    actuator[FLAP].pwm     = input.servos[0];

    // for computing control surface forces in for loop, so that you dont have to call each control surface separately
    csta[0]= AILERON;
    csta[1]= ELEVATOR;
    csta[2]= RUDDER_COMMON;

    s_motor.pwm_in[0] = input.servos[3]; // motor 1
    s_motor.pwm_in[1] = input.servos[4]; // motor 2
    s_motor.pwm_in[2] = input.servos[5]; // motor 3
    s_motor.pwm_in[3] = input.servos[6]; // motor 4
    s_motor.pwm_in[4] = input.servos[7]; // motor 5
    s_motor.pwm_in[5] = input.servos[8]; // motor 6
    s_motor.pwm_in[6] = input.servos[9]; // motor 7
    s_motor.pwm_in[7] = input.servos[10];// motor 8
}



