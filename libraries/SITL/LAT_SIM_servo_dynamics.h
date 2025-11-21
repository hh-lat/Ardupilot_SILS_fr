
#include <stdio.h>

#include "stdint.h"

typedef struct 
{
    int num_servos;
    /* data */
}S_SERVO_MANAGER;

extern S_SERVO_MANAGER s_servo_manager;

typedef enum
{
	// do not use 0 as enum for anytype
    NOT_ASSIGNED=0,
	AILERON_LEFT=1,
	AILERON_RIGHT=2,
	AILERON_COMMON=3,
	ELEVATOR_LEFT=4,
	ELEVATOR_RIGHT=5,
	ELEVATOR_COMMON=6,
	RUDDER_LEFT=7,
	RUDDER_RIGHT=8,
	RUDDER_COMMON=9,
	CANARD_LEFT=10,
	CANARD_RIGHT=11,
	FUSELAGE=12,
	SPOILER=13,
    FLAP=14,
    NOSE_LG_SERVO=15,
}CONTROL_SURFACE_TYPE;


typedef struct
{
    float pwm_in;
    float pwm_out;
    float angle;
    float angle_old;

    float pwm_min;
    float pwm_max;

    float angle_min;
    float angle_max;

    float ang_vel;
    float ang_accel;

    float omega;
    float zeta;

    float angular_rate_max;
    float angular_rate_min;

    float angular_accel_max;
    float angular_accel_min;
    CONTROL_SURFACE_TYPE type;

}S_SERVO;

extern S_SERVO s_servo[16];
extern CONTROL_SURFACE_TYPE csta[16];//control_surface_type_array


extern void v_servo_dynamics(float t_step_act);
extern void v_apply_rate_limits_to_control_surfaces(float);

extern void v_set_servo_params(float pwm_min, float pwm_max, float angle_min, float angle_max,
		float omega, float zeta, float min_rate, float max_rate,
		float min_accel, float max_accel,CONTROL_SURFACE_TYPE type);
extern void v_pwm_out_servo_to_angles();
