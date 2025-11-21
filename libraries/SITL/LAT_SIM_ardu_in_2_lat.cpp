
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
    s_servo[AILERON_LEFT].pwm_in    = input.servos[0]; // aileron left
    s_servo[AILERON_RIGHT].pwm_in   = input.servos[1]; // aileron right
    s_servo[ELEVATOR_COMMON].pwm_in = input.servos[2]; //elevator common
    s_servo[RUDDER_COMMON].pwm_in   = input.servos[2]; //elevator common
    s_servo[FLAP].pwm_in            = input.servos[3]; //rudder common
    s_servo[NOSE_LG_SERVO].pwm_in   = input.servos[4];

    s_motor[0].pwm_in = input.servos[2]; // motor 1
    s_motor[1].pwm_in = input.servos[2]; // motor 2
    s_motor[2].pwm_in = input.servos[2]; // motor 3
    s_motor[3].pwm_in = input.servos[2]; // motor 4
    s_motor[4].pwm_in = input.servos[2]; // motor 5
    s_motor[5].pwm_in = input.servos[2]; // motor 6
    s_motor[6].pwm_in = input.servos[2]; // motor 7
    s_motor[7].pwm_in = input.servos[2]; // motor 8

}



