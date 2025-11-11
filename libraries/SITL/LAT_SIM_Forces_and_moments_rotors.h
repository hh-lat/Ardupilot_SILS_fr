#include <plant.h>
#include "math_util.h"
#include "math.h"



extern float all_rotors_force[3];
extern float all_rotors_moment[3];

extern float rotor_xyz[28][3];
extern float rotor_force[28][3];
extern float rotor_moment[28][3];
extern float rotor_tilt[28][3];
extern float rotor_yaw_moment_b[28][3];

extern float rotor_r_direction[28];
extern float rotor_yaw_moment_m[28];


extern void fn_rotors_force_and_moments();
extern void fn_thrust_rotor2body();

extern void fn_yaw_moment_motor_frame();
extern void fn_yaw_moment_rotor2body();
extern void fn_moment_rotor2body();
