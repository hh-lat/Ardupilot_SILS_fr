
#include <stdio.h>
#include <math.h>
#include <AP_HAL/AP_HAL.h>
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
	double Vdot = 0.0;
	double VMag = 0.0;

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
	float friction_coff = 0.001;

	v_rotation_matrices_update(vehcle.phi,  vehcle.theta,  vehcle.psi,  vehcle.alpha,  vehcle.beta);

	 //updates J, RPM, Cmu based on throttle inputs & V_inf
	 //updates vehcle.all_rotors_force[] and vehcle.all_rotors_moment[] in body frame
	v_rotor_dynamics(vehcle.step_dt);

	v_update_vehcle_Cmu();

	v_aero_force_and_moments();// updates vehcle.all_aero_force[] and vehcle.all_aero_moment[]

	mass_inv =  1.0f/mass;

	vehcle.mg_b[0] = -mass*g*sinf(vehcle.theta);//- mass*g*cosf(vehcle.theta)*cosf(vehcle.phi);
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

	float force_arr_bd[3] = {fx, fy, fz};
	float force_arr_ned[3]={0.0};
	body_to_NED(force_arr_bd,  force_arr_ned);

	switch (vehcle.plane_moving_state)
	{
		case STATIONARY:
		{
			dydt[0] = 0.0f;
			dydt[1] = 0.0f;
			dydt[2] = 0.0f;
			dydt[3] = 0.0f;
			dydt[4] = 0.0f;
			dydt[5] = 0.0f;
			dydt[6] = 0.0f;
			dydt[7] = 0.0f;
			dydt[8] = 0.0f;
			dydt[9] = 0.0f;
			dydt[10] = 0.0f;
			dydt[11] = 0.0f;

			vehcle.p_dot =0;
			vehcle.q_dot =0;
			vehcle.r_dot =0;

			vehcle.FLG_NR = vehcle.mass*vehcle.g / (1.0 + fabsf(vehcle.FLG_x/vehcle.MLG_x));
			vehcle.MLG_NR = vehcle.mass*vehcle.g - vehcle.FLG_NR;

			vehcle.total_force_bd[0] = 0;
			vehcle.total_force_bd[1] = 0;
			vehcle.total_force_bd[2] = 0;
			vehcle.total_moment_bd[0] = 0;
			vehcle.total_moment_bd[1] = 0;
			vehcle.total_moment_bd[2] = 0;
		break;
		}
		
		case CT_RUNWAY_MOVING:
		{		
			//vehcle.plane_on_ground = 0;
			vehcle.FLG_NR = (force_arr_ned[2]*vehcle.MLG_x - m)/(vehcle.FLG_x + vehcle.MLG_x);
			vehcle.MLG_NR = (force_arr_ned[2] - vehcle.FLG_NR);

			if (vehcle.MLG_NR < 0.0f && vehcle.FLG_NR < 0.0f)
			{
				vehcle.FLG_NR = 0.0f;
				vehcle.plane_moving_state = IN_AIR;
			}
			else if (vehcle.FLG_NR < 0.0f && vehcle.MLG_NR>0.0f)
			{
				vehcle.FLG_NR = 0.0f;
				vehcle.plane_moving_state = CT_RUNWAY_ROTATING;
			}

			if (vehcle.FLG_NR < 0.0f)
			{
				vehcle.FLG_NR = 0.0f;
			}

			if (vehcle.MLG_NR < 0.0f)
			{
				vehcle.MLG_NR = 0.0f;
			}

			m = 0;//m + vehcle.FLG_NR*vehcle.FLG_x - vehcle.MLG_NR*vehcle.MLG_x;

			fx = fx - friction_coff*(vehcle.FLG_NR + vehcle.MLG_NR);
			dydt[0] = ((fx * mass_inv) + (r * v) - (q * w));//%u_dot//optimization
			dydt[1] = 0*((fy * mass_inv) + (p * w) - (r * u));//%v_dot
			dydt[2] = 0*((fz * mass_inv) + (q * u) - (p * v));  //%w_dot

			dydt[3] = 0*((Iyz * Iyz - Iy * Iz) * (l + q * (Ixz * p + Iyz * q - Iz * r) - r * (Ixy * p - Iy * q + Iyz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixz*Iy + Ixy*Iyz)*(n + p*(Ixy*p - Iy*q + Iyz*r) - q*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iyz + Ixy*Iz)*(m - p*(Ixz*p + Iyz*q - Iz*r) + r*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
			dydt[4] = 0*((Ixz * Ixz - Ix * Iz) * (m - p * (Ixz * p + Iyz * q - Iz * r) + r * (Ixy * q - Ix * p + Ixz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixy*Ixz + Ix*Iyz)*(n + p*(Ixy*p - Iy*q + Iyz*r) - q*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iyz + Ixy*Iz)*(l + q*(Ixz*p + Iyz*q - Iz*r) - r*(Ixy*p - Iy*q + Iyz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
			dydt[5] = 0*((Ixy * Ixy - Ix * Iy) * (n + p * (Ixy * p - Iy * q + Iyz * r) - q * (Ixy * q - Ix * p + Ixz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixy*Ixz + Ix*Iyz)*(m - p*(Ixz*p + Iyz*q - Iz*r) + r*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iy + Ixy*Iyz)*(l + q*(Ixz*p + Iyz*q - Iz*r) - r*(Ixy*p - Iy*q + Iyz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
			
			dydt[6] = 0*(p + (q * sinf(phi) * tanf(theta)) + (r * cosf(phi) * tanf(theta)));//%phi_dot
			dydt[7] = 0*((q * cosf(phi)) - (r * sinf(phi)));//%theta_dot
			dydt[8] = 0*((q * sinf(phi) * sec(theta)) + (r * cosf(phi) * sec(theta)));//%psi_dot

			vehcle.p_dot =0*dydt[3];
			vehcle.q_dot =dydt[4];
			vehcle.r_dot =0*dydt[5];

			body_to_NED(vehcle.V_b_gnd,  V);

			dydt[9]  = V[0];//%inertial velocity X
			dydt[10] = V[1];//%inertial velocity Y
			dydt[11] = V[2];//%inertial velocity Z

			vehcle.total_force_bd[0] = fx;
			vehcle.total_force_bd[1] = 0;
			vehcle.total_force_bd[2] = 0;
			vehcle.total_moment_bd[0] = 0;
			vehcle.total_moment_bd[1] = m;
			vehcle.total_moment_bd[2] = 0;
			
			break;
		}

		case CT_RUNWAY_ROTATING:
		{
        
		                                                                                                                            
		// 	if(1)
		// {
			VMag = sqrtf(u*u + v*v + w*w);

			vehcle.MLG_NR = vehcle.mass*vehcle.g - vehcle.all_lift_force - vehcle.all_rotors_force[0]*sinf(theta)
			                + vehcle.mass*vehcle.MLG_x*(vehcle.q_dot*cosf(theta) - q*q*sinf(theta));

		    Vdot = (1/vehcle.mass)*(vehcle.all_rotors_force[0] *cosf(theta)
		                              - vehcle.all_drag_force -  friction_coff*vehcle.MLG_NR);

			double Ieff_runway_rotating = Iy + vehcle.mass*(vehcle.MLG_x*vehcle.MLG_x + vehcle.MLG_z*vehcle.MLG_z)
			                              + vehcle.mass*(vehcle.MLG_x*vehcle.MLG_x)*cosf(theta)*cosf(theta);

			if (vehcle.MLG_NR < 0.0f)
			{
				vehcle.MLG_NR = 0.0f;
				vehcle.plane_moving_state = IN_AIR;
			}
			else if (vehcle.MLG_NR > 0.0f && vehcle.theta < vehcle.theta_tolerance_for_ground)
			{
				vehcle.MLG_NR = 0.0f;
				vehcle.plane_moving_state = CT_RUNWAY_MOVING;
			}

		
			dydt[0] = Vdot*cosf(theta) - VMag*q*sinf(theta);//%u_dot//optimization
			dydt[1] = 0*((fy * mass_inv) + (p * w) - (r * u));//%v_dot
			dydt[2] = Vdot*sinf(theta) + VMag*q*cosf(theta);  //%w_dot

			dydt[3] = 0*((Iyz * Iyz - Iy * Iz) * (l + q * (Ixz * p + Iyz * q - Iz * r) - r * (Ixy * p - Iy * q + Iyz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixz*Iy + Ixy*Iyz)*(n + p*(Ixy*p - Iy*q + Iyz*r) - q*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iyz + Ixy*Iz)*(m - p*(Ixz*p + Iyz*q - Iz*r) + r*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
			dydt[4] = (1/Ieff_runway_rotating)*(vehcle.all_aero_moment[1] + vehcle.all_lift_force*vehcle.MLG_x*cosf(theta)
                         + vehcle.all_rotors_force[0]*cosf(theta)*0.027
			             - vehcle.mass*vehcle.g*vehcle.MLG_x*cosf(theta)
						 + vehcle.mass*Vdot*vehcle.MLG_x*sinf(theta)
						 + vehcle.mass*vehcle.MLG_x*vehcle.MLG_x*q*q*sinf(theta)*cosf(theta));

			dydt[5] = 0*((Ixy * Ixy - Ix * Iy) * (n + p * (Ixy * p - Iy * q + Iyz * r) - q * (Ixy * q - Ix * p + Ixz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixy*Ixz + Ix*Iyz)*(m - p*(Ixz*p + Iyz*q - Iz*r) + r*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iy + Ixy*Iyz)*(l + q*(Ixz*p + Iyz*q - Iz*r) - r*(Ixy*p - Iy*q + Iyz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
			
			dydt[6] = 0*(p + (q * sinf(phi) * tanf(theta)) + (r * cosf(phi) * tanf(theta)));//%phi_dot
			dydt[7] = q;//((q * cosf(phi)) - (r * sinf(phi)));//%theta_dot
			dydt[8] = 0*((q * sinf(phi) * sec(theta)) + (r * cosf(phi) * sec(theta)));//%psi_dot

			body_to_NED(vehcle.V_b_gnd,  V);

			dydt[9]  = V[0];//%inertial velocity X
			dydt[10] = V[1];//%inertial velocity Y
			dydt[11] = V[2];//%inertial velocity Z
		// }
		// else
		// {
		// 	float A=0,B=0,C=0;
		// 	float FZ =0;

		// 	float temp3[3]={0};
		// 	float temp4[3]={0};

		// 	temp3[0] = fx;
		// 	temp3[1] = 0;
		// 	temp3[2] = fz;

		// 	body_to_NED(temp3, temp4);

		// 	FZ = temp4[2];

		// 	C = (mass*A*vehcle.MLG_x -1);
		// 	if (fabsf(C)<0.0001)
		// 	{
		// 		C=0.0001; // to avoid division by zero
		// 	}

		// 	A = -(vehcle.MLG_x*cosf(theta) - vehcle.MLG_z*sinf(theta));
		// 	A = A/Iy;
		// 	B = vehcle.MLG_x*sinf(theta) + vehcle.MLG_z*cosf(theta);

		// 	vehcle.MLG_NR = (mass*A*m + mass*B*q*q - FZ)/(C);

		// 	if (vehcle.MLG_NR < 0.0f)
		// 	{
		// 		vehcle.MLG_NR = 0.0f;
		// 		vehcle.plane_moving_state = IN_AIR;
		// 	}
		// 	else if (vehcle.MLG_NR > 0.0f && vehcle.theta < vehcle.theta_tolerance_for_ground)
		// 	{
		// 		vehcle.MLG_NR = 0.0f;
		// 		vehcle.plane_moving_state = CT_RUNWAY_MOVING;
		// 	}

		// 	fx = fx + vehcle.MLG_NR*sinf(theta);
		// 	fz = fz - vehcle.MLG_NR*cosf(theta);

		// 	m = m + vehcle.FLG_NR*vehcle.FLG_x - vehcle.MLG_NR*vehcle.MLG_x;

		// 	fx = fx - friction_coff*(vehcle.FLG_NR + vehcle.MLG_NR);

        //                 dydt[0] = ((fx * mass_inv) + (r * v) - (q * w));//%u_dot//optimization
		// 	dydt[1] = 0*((fy * mass_inv) + (p * w) - (r * u));//%v_dot
		// 	dydt[2] = ((fz * mass_inv) + (q * u) - (p * v));  //%w_dot

		// 	dydt[3] = 0*((Iyz * Iyz - Iy * Iz) * (l + q * (Ixz * p + Iyz * q - Iz * r) - r * (Ixy * p - Iy * q + Iyz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixz*Iy + Ixy*Iyz)*(n + p*(Ixy*p - Iy*q + Iyz*r) - q*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iyz + Ixy*Iz)*(m - p*(Ixz*p + Iyz*q - Iz*r) + r*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
		// 	dydt[4] =  ((Ixz * Ixz - Ix * Iz) * (m - p * (Ixz * p + Iyz * q - Iz * r) + r * (Ixy * q - Ix * p + Ixz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixy*Ixz + Ix*Iyz)*(n + p*(Ixy*p - Iy*q + Iyz*r) - q*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iyz + Ixy*Iz)*(l + q*(Ixz*p + Iyz*q - Iz*r) - r*(Ixy*p - Iy*q + Iyz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);

		// 	dydt[5] = 0*((Ixy * Ixy - Ix * Iy) * (n + p * (Ixy * p - Iy * q + Iyz * r) - q * (Ixy * q - Ix * p + Ixz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixy*Ixz + Ix*Iyz)*(m - p*(Ixz*p + Iyz*q - Iz*r) + r*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iy + Ixy*Iyz)*(l + q*(Ixz*p + Iyz*q - Iz*r) - r*(Ixy*p - Iy*q + Iyz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
			
		// 	dydt[6] = 0*(p + (q * sinf(phi) * tanf(theta)) + (r * cosf(phi) * tanf(theta)));//%phi_dot
		// 	dydt[7] = ((q * cosf(phi)) - (r * sinf(phi)));//%theta_dot
		// 	dydt[8] = 0*((q * sinf(phi) * sec(theta)) + (r * cosf(phi) * sec(theta)));//%psi_dot

		// 	body_to_NED(vehcle.V_b_gnd,  V);

		// 	dydt[9]  = V[0];//%inertial velocity X
		// 	dydt[10] = V[1];//%inertial velocity Y
		// 	dydt[11] = V[2];//%inertial velocity Z
		// }
			
			vehcle.p_dot =0*dydt[3];
			vehcle.q_dot =dydt[4];
			vehcle.r_dot =0*dydt[5];

			vehcle.total_force_bd[0] = fx - vehcle.MLG_NR*sinf(theta);
			vehcle.total_force_bd[1] = 0;
			vehcle.total_force_bd[2] = fz + vehcle.MLG_NR*cosf(theta);
			vehcle.total_moment_bd[0] = 0;
			vehcle.total_moment_bd[1] = m;
			vehcle.total_moment_bd[2] = 0;

			break;	
		}

		case IN_AIR:
		{
			//vehcle.plane_on_ground = 0;
			vehcle.MLG_NR = 0;
			vehcle.FLG_NR = 0;

			m = m + vehcle.FLG_NR*vehcle.FLG_x - vehcle.MLG_NR*vehcle.MLG_x;

			if (vehcle.alt_agl < vehcle.altitude_tolerance_for_ground && vehcle.theta < vehcle.theta_tolerance_for_ground)
			{
				vehcle.plane_moving_state = CT_RUNWAY_MOVING;
			}
			else if (vehcle.alt_agl < vehcle.altitude_tolerance_for_ground)
			{
				vehcle.plane_moving_state = CT_RUNWAY_ROTATING;
			}

			// static uint32_t last_time_ms = 0;
			// // Get current runtime in milliseconds from ArduPilot HAL
			// if (last_time_ms == 0)
			// {
			// 	last_time_ms = AP_HAL::millis();
			// }

			// if (AP_HAL::millis() - last_time_ms < 50)
			// {
			// 	m=0;
			// 	l=0;
			// 	n=0;
			// }

			// vehcle.Cm = vehcle.Cm_0 + vehcle.Cm_alpha*vehcle.alpha + vehcle.Cm_delta_e*vehcle.delta_e +
			// 		 vehcle.Cm_q*vehcle.q*vehcle.c/(2.0f*vehcle.tas);
			// 		 //vehcle.Cm_Cmu*vehcle.Cmu + vehcle.Cm_alpha_Cmu*vehcle.alpha*vehcle.Cmu +
			// 		 //vehcle.Cm_delta_f*vehcle.delta_f;
			// m = vehcle.Q*vehcle.s*vehcle.c*(vehcle.Cm) + vehcle.all_rotors_moment[1] ;
			// m = (0.5*1.15*25*25)*vehcle.s*vehcle.c*(vehcle.Cm) + vehcle.all_rotors_moment[1] ;

			dydt[0] = ((fx * mass_inv) + (r * v) - (q * w));//%u_dot//optimization
			dydt[1] = ((fy * mass_inv) + (p * w) - (r * u));//%v_dot
			dydt[2] = ((fz * mass_inv) + (q * u) - (p * v));  //%w_dot

			dydt[3] = ((Iyz * Iyz - Iy * Iz) * (l + q * (Ixz * p + Iyz * q - Iz * r) - r * (Ixy * p - Iy * q + Iyz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixz*Iy + Ixy*Iyz)*(n + p*(Ixy*p - Iy*q + Iyz*r) - q*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iyz + Ixy*Iz)*(m - p*(Ixz*p + Iyz*q - Iz*r) + r*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
			dydt[4] = ((Ixz * Ixz - Ix * Iz) * (m - p * (Ixz * p + Iyz * q - Iz * r) + r * (Ixy * q - Ix * p + Ixz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixy*Ixz + Ix*Iyz)*(n + p*(Ixy*p - Iy*q + Iyz*r) - q*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iyz + Ixy*Iz)*(l + q*(Ixz*p + Iyz*q - Iz*r) - r*(Ixy*p - Iy*q + Iyz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
			dydt[5] = ((Ixy * Ixy - Ix * Iy) * (n + p * (Ixy * p - Iy * q + Iyz * r) - q * (Ixy * q - Ix * p + Ixz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixy*Ixz + Ix*Iyz)*(m - p*(Ixz*p + Iyz*q - Iz*r) + r*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iy + Ixy*Iyz)*(l + q*(Ixz*p + Iyz*q - Iz*r) - r*(Ixy*p - Iy*q + Iyz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
			
			dydt[6] = (p + (q * sinf(phi) * tanf(theta)) + (r * cosf(phi) * tanf(theta)));//%phi_dot
			dydt[7] = ((q * cosf(phi)) - (r * sinf(phi)));//%theta_dot
			dydt[8] = ((q * sinf(phi) * sec(theta)) + (r * cosf(phi) * sec(theta)));//%psi_dot

			body_to_NED(vehcle.V_b_gnd,  V);

			vehcle.p_dot =dydt[3];
			vehcle.q_dot =dydt[4];
			vehcle.r_dot =dydt[5];

			// if (fabsf(vehcle.delta_e) > 5/57.3)
			// {
			// 	vehcle.q_dot =10*(0-vehcle.q) + (vehcle.Cm_delta_e*(vehcle.delta_e))*0.5;//dydt[4];
			// }
			// else 
			// {
			// 	vehcle.q_dot =10*(0-vehcle.q) ;
			// }
			

			dydt[9]  = V[0];//%inertial velocity X
			dydt[10] = V[1];//%inertial velocity Y
			dydt[11] = V[2];//%inertial velocity Z

			vehcle.total_force_bd[0] = fx;
			vehcle.total_force_bd[1] = fy;
			vehcle.total_force_bd[2] = fz;
			vehcle.total_moment_bd[0] = l;
			vehcle.total_moment_bd[1] = m;
			vehcle.total_moment_bd[2] = n;
			break;	
		}

		default:
		{
			// normal in air condition
			break;		
		}
	}
}

