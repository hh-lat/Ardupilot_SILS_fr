
#include "LAT_SIM_math_util.h"
#include "LAT_SIM_Runner.h"
#include "LAT_SIM_Forces_and_moments_ctrl_srfce.h"
#include "LAT_SIM_Conversions_Frame_rotations.h"
#include "LAT_SIM_derivative.h"
#include "LAT_SIM_rotor_dynamics.h"
#include "LAT_SIM_servo_dynamics.h"


void v_derivative(float Plane_state[],float t,float dydt[])
{
	float u,v,w,p,q,r,phi,theta;
	float Ix,Iy,Iz,Ixz,Ixy=0,Iyz=0;
	float mass_inv;
	float l,m,n;
	float fx,fy,fz;
	float V[3];
	float mass,g;

	u = Plane_state[0];
	v = Plane_state[1];
	w = Plane_state[2];

	p = Plane_state[3];
	q = Plane_state[4];
	r = Plane_state[5];

	phi   = Plane_state[6];
	theta = Plane_state[7];

	Ix=vehcle.Ixx;
	Iy=vehcle.Iyy;
	Iz=vehcle.Izz;
	Ixz=vehcle.Ixz;
	Iyz=vehcle.Iyz;
	Ixy=vehcle.Ixy;
	g= vehcle.g;
	mass = vehcle.mass;

	//calculate density using Indian atmosphere model
	// if (Plane_state[11] > 0.0)
	// {
	// 	atmind(1.0f, &vehcle.pressure, &vehcle.sound_speed, &vehcle.rho); //for positive z component, which means going below the ground directly cosider z as '1'
	// }
	// else
	// {
	// 	atmind(fabsf(Plane_state[11]), &vehcle.pressure, &vehcle.sound_speed, &vehcle.rho); //atmind function call. Use of fabsf inplace of norm
	// }
	
	vehcle.rho = 1.15;

	v_rotation_matrices_update( vehcle.phi,  vehcle.theta,  vehcle.psi,  vehcle.alpha,  vehcle.beta);

	//v_update_vehcle_states(Plane_state);
	v_rotors_force_and_moments(); //located in rotor_dynamics

	v_aero_force_and_moments(); //located in Forces_and_moments_ctrl_srfce.c line:89


	mass_inv = 1.0f/mass;

	vehcle.mg_b[0] = -mass*g*sinf(vehcle.theta);
	vehcle.mg_b[1] =  mass*g*cosf(vehcle.theta)*sinf(vehcle.phi);
	vehcle.mg_b[2] =  mass*g*cosf(vehcle.theta)*cosf(vehcle.phi);

	l = vehcle.all_aero_moment[0] + vehcle.all_rotors_moment[0] +  0.0*vehcle.all_payload_moment[0];
	m = vehcle.all_aero_moment[1] + vehcle.all_rotors_moment[1] +  0.0*vehcle.all_payload_moment[1];
	n = vehcle.all_aero_moment[2] + vehcle.all_rotors_moment[2] +  0.0*vehcle.all_payload_moment[2];    //optimization

	fx = vehcle.all_rotors_force[0] + vehcle.all_aero_force[0] + vehcle.mg_b[0]  + 0.0*vehcle.all_payload_force[0];
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	fy = vehcle.all_rotors_force[1] + vehcle.all_aero_force[1] + vehcle.mg_b[1]  + 0.0*vehcle.all_payload_force[1];
	//	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	fz = vehcle.all_rotors_force[2] + vehcle.all_aero_force[2] + vehcle.mg_b[2]  + 0.0*vehcle.all_payload_force[2];

	dydt[0] = ((fx * mass_inv) + (r * v) - (q * w));//%u_dot//optimization
	dydt[1] = ((fy * mass_inv) + (p * w) - (r * u));//%v_dot
	dydt[2] = ((fz * mass_inv) + (q * u) - (p * v));  //%w_dot

	//	dydt[3] = ((c1*p*q) - (c2*q*r) + (c3*l) + (c4*n));  //%p_dot   % valid for Plane symmetric abt xz plane, i.e Iyz=0, Ixy=0;
	//	dydt[4] = (c5*p*r) - (c6*((p*p)-(r*r)))  + (m/Iy);  //%q_dot
	//  dydt[5] = ((c7*p*q) - (c1*q*r) + (c4*l) + (c8*n));  //%r_dot

	dydt[3] = ((Iyz * Iyz - Iy * Iz) * (l + q * (Ixz * p + Iyz * q - Iz * r) - r * (Ixy * p - Iy * q + Iyz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixz*Iy + Ixy*Iyz)*(n + p*(Ixy*p - Iy*q + Iyz*r) - q*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iyz + Ixy*Iz)*(m - p*(Ixz*p + Iyz*q - Iz*r) + r*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
	dydt[4] = ((Ixz * Ixz - Ix * Iz) * (m - p * (Ixz * p + Iyz * q - Iz * r) + r * (Ixy * q - Ix * p + Ixz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixy*Ixz + Ix*Iyz)*(n + p*(Ixy*p - Iy*q + Iyz*r) - q*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iyz + Ixy*Iz)*(l + q*(Ixz*p + Iyz*q - Iz*r) - r*(Ixy*p - Iy*q + Iyz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
	dydt[5] = ((Ixy * Ixy - Ix * Iy) * (n + p * (Ixy * p - Iy * q + Iyz * r) - q * (Ixy * q - Ix * p + Ixz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixy*Ixz + Ix*Iyz)*(m - p*(Ixz*p + Iyz*q - Iz*r) + r*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iy + Ixy*Iyz)*(l + q*(Ixz*p + Iyz*q - Iz*r) - r*(Ixy*p - Iy*q + Iyz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);

	dydt[6] = (p + (q * sinf(phi) * tanf(theta)) + (r * cosf(phi) * tanf(theta)));//%phi_dot
	dydt[7] = (q * cosf(phi)) - (r * sinf(phi));//%theta_dot
	dydt[8] = ((q * sinf(phi) * sec(theta)) + (r * cosf(phi) * sec(theta)));//%psi_dot

	body_to_NED(vehcle.V_b_gnd,  V);

	dydt[9]  = V[0];//%inertial velocity X
	dydt[10] = V[1];//%inertial velocity Y
	dydt[11] = V[2];//%inertial velocity Z

	vehcle.Accel_b[0] = fx - vehcle.mg_b[0];// for ardupilot update_dynamics
	vehcle.Accel_b[1] = fy - vehcle.mg_b[1];
	vehcle.Accel_b[2] = fz - vehcle.mg_b[2];

}

