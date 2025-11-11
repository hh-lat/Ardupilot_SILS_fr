#include <math.h>
#include <stdlib.h>

#include <aero.h>
#include <math_util.h>
#include <common_variable.h>
#include "plant.h"
#include "Ground_model.h"
#include "Forces_and_moments_ctrl_srfce.h"
#include "Forces_and_moments_rotors.h"
#include "Conversions_Frame_rotations.h"
#include "update_vehicle_state.h"
#include "Payload_force_and_moment.h"


void derivative(float Plane_state[],float t,float dydt[],float acc_real_plant[])
{
	float u,v,w,p,q,r,phi,theta,psi;
	float mass_inv;
	float l,m,n;
	float fx,fy,fz;
	float V[3];

	u = Plane_state[0];
	v = Plane_state[1];
	w = Plane_state[2];

	p = Plane_state[3];
	q = Plane_state[4];
	r = Plane_state[5];

	phi   = Plane_state[6];
	theta = Plane_state[7];
	psi   = Plane_state[8];

	//calculate density using Indian atmosphere model
	if (Plane_state[11] > 0.0)
	{
		atmind(1.0f, &vehicle.pressure, &vehicle.sound_speed, &vehicle.rho); //for positive z component, which means going below the ground directly cosider z as '1'
	}
	else
	{
		atmind(fabsf(Plane_state[11]), &vehicle.pressure, &vehicle.sound_speed, &vehicle.rho); //atmind function call. Use of fabsf inplace of norm
	}
	
	Rotation_matrices_update( vehicle.phi,  vehicle.theta,  vehicle.psi,  vehicle.alpha,  vehicle.beta);

	v_update_vehicle_states(Plane_state);

	v_aero_force_and_moments(); //located in Forces_and_moments_ctrl_srfce.c line:89

	fn_rotors_force_and_moments(); //located in Forces_and_moments_rotors.c line:89

	Payload_forces_and_moments_body_frame(); //located in Payload_force_and_moments.c line:6

	mass_inv = 1.0f/mass;

	vehicle.mg_b[0] = -mass*g*sinf(vehicle.theta);
	vehicle.mg_b[1] =  mass*g*cosf(vehicle.theta)*sinf(vehicle.phi);
	vehicle.mg_b[2] =  mass*g*cosf(vehicle.theta)*cosf(vehicle.phi);


	l = vehicle.all_aero_moment[0] + vehicle.all_rotors_moment[0] +  0.0*vehicle.all_payload_moment[0];
	m = vehicle.all_aero_moment[1] + vehicle.all_rotors_moment[1] +  0.0*vehicle.all_payload_moment[1];
	n = vehicle.all_aero_moment[2] + vehicle.all_rotors_moment[2] +  0.0*vehicle.all_payload_moment[2];    //optimization

	fx = vehicle.all_rotors_force[0] + vehicle.all_aero_force[0] + vehicle.mg_b[0]  + 0.0*vehicle.all_payload_force[0];
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	fy = vehicle.all_rotors_force[1] + vehicle.all_aero_force[1] + vehicle.mg_b[1]  + 0.0*vehicle.all_payload_force[1];
	//	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	fz = vehicle.all_rotors_force[2] + vehicle.all_aero_force[2] + vehicle.mg_b[2]  + 0.0*vehicle.all_payload_force[2];

	dydt[0] = ((fx * mass_inv) + (r * v) - (q * w));//%u_dot//optimization
	dydt[1] = ((fy * mass_inv) + (p * w) - (r * u));//%v_dot
	dydt[2] = ((fz * mass_inv) + (q * u) - (p * v));  //%w_dot


	//	dydt[3] = ((c1*p*q) - (c2*q*r) + (c3*l) + (c4*n));  //%p_dot   % valid for Plane symmetric abt xz plane, i.e Iyz=0, Ixy=0;
	//	dydt[4] = (c5*p*r) - (c6*((p*p)-(r*r)))  + (m/Iy);  //%q_dot
	//  dydt[5] = ((c7*p*q) - (c1*q*r) + (c4*l) + (c8*n));  //%r_dot

	// lines added on 24 April 2021 to make more generalized 6DOF for plane having nonzero Iyz and Ixy
	dydt[3] = ((Iyz * Iyz - Iy * Iz) * (l + q * (Ixz * p + Iyz * q - Iz * r) - r * (Ixy * p - Iy * q + Iyz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixz*Iy + Ixy*Iyz)*(n + p*(Ixy*p - Iy*q + Iyz*r) - q*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iyz + Ixy*Iz)*(m - p*(Ixz*p + Iyz*q - Iz*r) + r*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
	dydt[4] = ((Ixz * Ixz - Ix * Iz) * (m - p * (Ixz * p + Iyz * q - Iz * r) + r * (Ixy * q - Ix * p + Ixz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixy*Ixz + Ix*Iyz)*(n + p*(Ixy*p - Iy*q + Iyz*r) - q*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iyz + Ixy*Iz)*(l + q*(Ixz*p + Iyz*q - Iz*r) - r*(Ixy*p - Iy*q + Iyz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
	dydt[5] = ((Ixy * Ixy - Ix * Iy) * (n + p * (Ixy * p - Iy * q + Iyz * r) - q * (Ixy * q - Ix * p + Ixz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixy*Ixz + Ix*Iyz)*(m - p*(Ixz*p + Iyz*q - Iz*r) + r*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iy + Ixy*Iyz)*(l + q*(Ixz*p + Iyz*q - Iz*r) - r*(Ixy*p - Iy*q + Iyz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);

	dydt[6] = (p + (q * sinf(phi) * tanf(theta)) + (r * cosf(phi) * tanf(theta)));//%phi_dot
	dydt[7] = (q * cosf(phi)) - (r * sinf(phi));//%theta_dot
	dydt[8] = ((q * sinf(phi) * sec(theta)) + (r * cosf(phi) * sec(theta)));//%psi_dot


	body_to_NED(vehicle.V_b_gnd,  V);

	dydt[9]  = V[0];//%inertial velocity X
	dydt[10] = V[1];//%inertial velocity Y
	dydt[11] = V[2];//%inertial velocity Z

	dydt[12] = 0.0f;
	dydt[13] = 0.0f;
	dydt[14] = 0.0f;

	//	ax_real=(fx/mass)-(mg_b[0][0]/mass);//optimization
	//	ay_real=(fy/mass)-(mg_b[1][0]/mass);//optimization
	//	az_real=(fz/mass)-(mg_b[2][0]/mass);//optimization

	/*acc_real_plant[0]=((fx*mass_inv) -(mg_b[0][0]*mass_inv));// sending gravity subtracted acceleration to Pseudo_INS
	acc_real_plant[1]=((fy*mass_inv) -(mg_b[1][0]*mass_inv));
	acc_real_plant[2]=((fz*mass_inv) -(mg_b[2][0]*mass_inv));*/ /*commented on 26 may 2021, we will now send true accel to pseudo ins*/

	acc_real_plant[0] = (fx * mass_inv);//
	acc_real_plant[1] = (fy * mass_inv);// true acceleration in m/sec^2 , sent to Pseudo Ins, 26 may 2021
	acc_real_plant[2] = (fz * mass_inv);

}

