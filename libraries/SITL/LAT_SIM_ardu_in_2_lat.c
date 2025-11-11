

void  v_ardu_input_to_lat_input(struct sitl_input &input)
{
    v_map_ardu_in_for_equinox(input);
}



void v_map_ardu_in_for_equinox(struct sitl_input &input)
{
    actuator[AILERON].pwm      = input.servos[0];
    actuator[ELEVATOR].pwm     = input.servos[1];
    actuator[RUDDER].pwm       = input.servos[2];

    actuator[FLAP].pwm  = input.servos[0];// aileron left
    lat_input.rudder_common    = input.servos[3];
}