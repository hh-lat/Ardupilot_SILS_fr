#include <math.h>
#include "LAT_SIM_math_util.h"

#include "LAT_SIM_Forces_and_moments_ctrl_srfce.h"
#include "LAT_SIM_servo_dynamics.h"
#include "LAT_SIM_Runner.h"


#define DIV 1e-1

// =====================================================================
//  Characterized 2nd-order servo model (servo model card, run_20260611):
//      H(s) = K * wn^2 / (s^2 + 2*zeta*wn*s + wn^2) * e^(-tau*s)
//  Complete model = this linear TF (+) a slew-rate limiter.
//  Applied to elevator / rudder / aileron (uSTOL): the commanded angle from
//  the PWM->angle map is the INPUT (deg), and the model output is the ACTUAL
//  angle. We keep the fitted zeta and wn, and use a DC gain of 0.96 acting on
//  the already-mapped command angle (so steady-state actual = 0.96 * command).
// =====================================================================
#define SERVO_TF_WN       46.9f          // natural frequency [rad/s]  (fn = 7.47 Hz)
#define SERVO_TF_ZETA     0.756f         // damping ratio
#define SERVO_TF_DCGAIN   0.96f          // steady-state gain: actual = 0.96 * commanded
#define SERVO_TF_TAU      0.014f         // transport delay [s]
#define SERVO_TF_SUBSTEP  0.001f         // max internal integration step [s] (stability)
#define SERVO_TF_DELAY_N  512            // transport-delay ring-buffer length [samples]

// --- Load-dependent slew-rate limit ---------------------------------------
//  The slew rate falls linearly with the aerodynamic hinge moment on the
//  surface:  L = C_surface * deflection * q * S * chord/4   [N*m].
//  rate(L) = NOLOAD - (NOLOAD-FULLLOAD)*(|L|/L_ref), clamped to [FULLLOAD,NOLOAD].
#define SERVO_SLEW_NOLOAD   360.0f       // no-load slew rate [deg/s]
#define SERVO_SLEW_FULLLOAD  60.0f       // slew rate at/above the reference load [deg/s]
#define SERVO_LOAD_REF       2.4517f     // reference hinge moment for the 60 deg/s floor [N*m] (= 25 kg*cm)
#define SERVO_LOAD_C_TAIL    0.28f       // tail chord c_T [m]      (elevator hinge-load arm)
#define SERVO_LOAD_C_VTAIL   0.312f      // vertical-tail chord c_vt [m] (rudder hinge-load arm)
                                         // (ailerons use the wing chord, vehcle.c)

S_SERVO s_servo[16];
CONTROL_SURFACE_TYPE csta[16] ={NOT_ASSIGNED};
S_SERVO_MANAGER s_servo_manager;


void v_set_servo_params(float pwm_min, float pwm_max, float angle_min, float angle_max,
		float omega, float zeta, float min_rate, float max_rate,
		float min_accel, float max_accel, CONTROL_SURFACE_TYPE type)
{
	s_servo[type].pwm_min = pwm_min;
	s_servo[type].pwm_max = pwm_max;

	s_servo[type].angle_min = angle_min;
	s_servo[type].angle_max = angle_max;

	s_servo[type].omega = omega;
	s_servo[type].zeta = zeta;

	s_servo[type].angular_rate_min = min_rate;
	s_servo[type].angular_rate_max = max_rate;

	s_servo[type].angular_accel_min = min_accel;
	s_servo[type].angular_accel_max = max_accel;

	s_servo[type].type = type;
}

void v_pwm_out_servo_to_angles()
{
	uint8_t i = 0;

	for(i=0;i<s_servo_manager.num_servos;i++)
	{
		if (s_servo[i].pwm_out <= s_servo[i].pwm_min)
		{
			s_servo[i].angle = s_servo[i].angle_min;
		}
		else if (s_servo[i].pwm_out >= s_servo[i].pwm_max)
		{
			s_servo[i].angle = s_servo[i].angle_max;
		}
		else if ( (s_servo[i].pwm_out > s_servo[i].pwm_min) && (s_servo[i].pwm_out < s_servo[i].pwm_max) )
		{
			s_servo[i].angle = s_servo[i].angle_min +
					((s_servo[i].angle_max - s_servo[i].angle_min)/(s_servo[i].pwm_max - s_servo[i].pwm_min))*(s_servo[i].pwm_out - s_servo[i].pwm_min);
		}
	}
}

void v_pwm_in_2_out_servo()
{
	uint8_t i = 0;

	for(i=0;i<s_servo_manager.num_servos;i++)
	{
		if (s_servo[i].pwm_in <= s_servo[i].pwm_min)
		{
			s_servo[i].pwm_out = s_servo[i].pwm_min;
		}
		else if (s_servo[i].pwm_in >= s_servo[i].pwm_max)
		{
			s_servo[i].pwm_out = s_servo[i].pwm_max;
		}
		else if ( (s_servo[i].pwm_in > s_servo[i].pwm_min) && (s_servo[i].pwm_in < s_servo[i].pwm_max) )
		{
			s_servo[i].pwm_out = s_servo[i].pwm_in;
		}
	}
}


void v_servo_dynamics(float t_step_act)
{
	v_pwm_in_2_out_servo();
	v_pwm_out_servo_to_angles();
	v_apply_rate_limits_to_control_surfaces(t_step_act);
	v_fill_vehcle_angles();
}


void v_fill_vehcle_angles()
{
	vehcle.delta_a = s_servo[AILERON_COMMON].angle;
	vehcle.delta_e = s_servo[ELEVATOR_COMMON].angle;
	vehcle.delta_r = s_servo[RUDDER_COMMON].angle;

	vehcle.delta_aL = s_servo[AILERON_LEFT].angle;
	vehcle.delta_aR = s_servo[AILERON_RIGHT].angle;
	vehcle.delta_f = s_servo[FLAP].angle;
}



// Which surfaces use the characterized 2nd-order servo model (s_servo is indexed
// by CONTROL_SURFACE_TYPE, so the loop index IS the type).
static int v_servo_uses_tf_model(int idx)
{
	return (idx == AILERON_LEFT  || idx == AILERON_RIGHT || idx == AILERON_COMMON ||
	        idx == ELEVATOR_COMMON || idx == RUDDER_COMMON);
}

// Load-dependent slew-rate limit [rad/s] from the aerodynamic hinge moment on
// the surface: L = C_surface * deflection * q * S * chord/4. The effectiveness
// coefficients (vehcle.CL_delta_e / CL_delta_a / CY_delta_r) are the uSTOL
// "inherent" values written each frame by the aero force/moment functions.
static float v_servo_load_slew(int idx, float deflection)
{
	float coeff, chord;
	if (idx == ELEVATOR_COMMON)    { coeff = vehcle.CL_delta_e; chord = SERVO_LOAD_C_TAIL;  }
	else if (idx == RUDDER_COMMON) { coeff = vehcle.CY_delta_r; chord = SERVO_LOAD_C_VTAIL; }
	else                           { coeff = vehcle.CL_delta_a; chord = vehcle.c;           } // ailerons -> wing chord

	float load = fabsf(coeff*deflection) * vehcle.Q * vehcle.s * chord * 0.25f;   // hinge moment [N*m]
	float rate = SERVO_SLEW_NOLOAD - (SERVO_SLEW_NOLOAD - SERVO_SLEW_FULLLOAD)*(load/SERVO_LOAD_REF);
	if (rate > SERVO_SLEW_NOLOAD)   rate = SERVO_SLEW_NOLOAD;
	if (rate < SERVO_SLEW_FULLLOAD) rate = SERVO_SLEW_FULLLOAD;

	return rate*D2R;   // [rad/s]
}

// Advance one servo's 2nd-order model by dt: command angle (s->angle, from the
// PWM->angle map) -> transport delay -> 2nd-order TF -> load-dependent slew limit.
static void v_servo_tf_step(S_SERVO *s, int idx, float dt)
{
	const float wn   = SERVO_TF_WN;
	const float ze   = SERVO_TF_ZETA;
	const float K    = SERVO_TF_DCGAIN;

	float cmd = s->angle;   // commanded angle from the PWM->angle map [rad]
	s->angle_cmd = cmd;     // record commanded angle for logging

	// Seed the state on first use so the servo starts at the current command
	// instead of slewing up from zero.
	if (!s->tf_init)
	{
		s->tf_y    = cmd;
		s->tf_ydot = 0.0f;
		for (int k = 0; k < SERVO_TF_DELAY_N; k++) s->tf_delay_buf[k] = cmd;
		s->tf_head = 0;
		s->tf_init = 1;
	}

	// load-dependent slew-rate limit from the current surface deflection
	float slew = v_servo_load_slew(idx, s->tf_y);
	s->slew_used = slew;    // record applied slew limit for logging

	// Transport delay (tau): frame-based ring buffer; ndelay self-adjusts to dt
	// so the delay stays ~tau seconds regardless of the SITL frame rate.
	int ndelay = (int)lroundf(SERVO_TF_TAU / dt);
	if (ndelay < 0) ndelay = 0;
	if (ndelay > SERVO_TF_DELAY_N - 1) ndelay = SERVO_TF_DELAY_N - 1;
	s->tf_head = (s->tf_head + 1) % SERVO_TF_DELAY_N;
	s->tf_delay_buf[s->tf_head] = cmd;
	int ridx = (s->tf_head - ndelay + SERVO_TF_DELAY_N) % SERVO_TF_DELAY_N;
	float u = s->tf_delay_buf[ridx];   // delayed command (TF input)

	// Integrate  y'' + 2*ze*wn*y' + wn^2*y = K*wn^2*u  (steady state y = K*u).
	// Sub-stepped semi-implicit Euler so it stays stable for any SITL step.
	int nsub = (int)ceilf(dt / SERVO_TF_SUBSTEP);
	if (nsub < 1) nsub = 1;
	float h = dt / (float)nsub;
	for (int k = 0; k < nsub; k++)
	{
		float ydd = wn*wn*(K*u - s->tf_y) - 2.0f*ze*wn*s->tf_ydot;
		s->tf_ydot += ydd*h;
		// load-dependent slew-rate (output speed) limit [60..360 deg/s]
		if (s->tf_ydot >  slew) s->tf_ydot =  slew;
		else if (s->tf_ydot < -slew) s->tf_ydot = -slew;
		s->tf_y += s->tf_ydot*h;
	}

	s->ang_vel   = s->tf_ydot;
	s->angle     = s->tf_y;     // actual angle -> consumed by v_fill_vehcle_angles
	s->angle_old = s->tf_y;
}

void v_apply_rate_limits_to_control_surfaces(float t_step_act)
{
	uint8_t i = 0;
	float angle_rate_cmd = 0.0f;

	for(i=0;i<s_servo_manager.num_servos;i++)
	{
		// uSTOL elevator/rudder/aileron use the characterized 2nd-order servo
		// model (TF + transport delay + slew limit) instead of the simple
		// first-order rate clamp.
		if (vehcle.plane_model == PLANE_USTOL_V1 && v_servo_uses_tf_model(i))
		{
			v_servo_tf_step(&s_servo[i], (int)i, t_step_act);
			continue;
		}

		angle_rate_cmd = (s_servo[i].angle - s_servo[i].angle_old)/t_step_act;

		if (angle_rate_cmd > s_servo[i].angular_rate_max)
		{
			angle_rate_cmd = s_servo[i].angular_rate_max;
		}
		else if (angle_rate_cmd < s_servo[i].angular_rate_min)
		{
			angle_rate_cmd = s_servo[i].angular_rate_min;
		}

		s_servo[i].angle = s_servo[i].angle_old + angle_rate_cmd*t_step_act;

		s_servo[i].angle_old = s_servo[i].angle;
	}
}
