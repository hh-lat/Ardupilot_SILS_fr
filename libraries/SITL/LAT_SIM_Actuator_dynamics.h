
#include <stdio.h>


typedef enum
{
	// do not use 0 as enum for anytype
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
}AKAP_CONTROL_SURFACE_TYPE;



typedef struct
{
    float cL;
    float cD;
    float cY;
    float cl;
    float cm;
    float cn;

    float pwm;
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

    float max_rate;
    float min_rate;

    float max_accel;
    float min_accel;
    AKAP_CONTROL_SURFACE_TYPE type;

}Ctrl_srfc_actuator;

extern Ctrl_srfc_actuator actuator[16];
extern AKAP_CONTROL_SURFACE_TYPE csta[16];//control_surface_type_array


extern void Actuator_dynamics(float t_step_act);
extern void v_fill_pwm_in_csta();


