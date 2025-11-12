
#include "LAT_SIM_ardu_in_2_lat.h"
#include "LAT_SIM_Actuator_dynamics.h"

void v_lat_fdm_run()
{
    float t_step = 0.001;
    Actuator_dynamics(t_step); // converts servopwm to angles and applies rate limits
   	v_motor_pwm_to_throttle(t_step); // throttle from pwm and applies rate limit on throttle

    rotor_dynamics(t_step); // updates rotor speeds and thrust/torque

    


}