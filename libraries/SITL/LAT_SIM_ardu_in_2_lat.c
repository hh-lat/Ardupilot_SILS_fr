
#include "LAT_SIM_ardu_in_2_lat.h"
#include "LAT_SIM_servo_dynamics.h"
#include "LAT_SIM_rotor_dynamics.h"
#include "LAT_SIM_Runner.h"

void  v_ardu_input_to_lat_input(struct sitl_input input)
{
    v_map_ardu_in_for_equinox(input);
}



void v_map_ardu_in_for_equinox(struct sitl_input input)
{
    s_servo[].pwm_delta_aR = input.servos[0]; // aileron left
    s_servo[].pwm_delta_aL = input.servos[1]; // aileron right
    s_servo[].pwm_delta_eR = input.servos[2]; //elevator common
    s_servo[].pwm_delta_eL = input.servos[2]; //elevator common

    s_servo.pwm_delta_fR = input.servos[3]; //rudder common
    s_servo.pwm_delta_fL = input.servos[3]; //rudder common

    s_servo.pwm_delta_rR = input.servos[3]; // no flaps
    s_servo.pwm_delta_rL = input.servos[3]; // no flaps

    s_servo.pwm_delta_nlg    = input.servos[4];
    s_servo.pwm_delta_brakes = input.servos[5];

    s_motor.pwm_in_esc[0] = input.servos[2]; // motor 1
    s_motor.pwm_in_esc[1] = input.servos[2]; // motor 2
    s_motor.pwm_in_esc[2] = input.servos[2]; // motor 3
    s_motor.pwm_in_esc[3] = input.servos[2]; // motor 4
    s_motor.pwm_in_esc[4] = input.servos[2]; // motor 5
    s_motor.pwm_in_esc[5] = input.servos[2]; // motor 6
    s_motor.pwm_in_esc[6] = input.servos[2]; // motor 7
    s_motor.pwm_in_esc[7] = input.servos[2]; // motor 8
}



