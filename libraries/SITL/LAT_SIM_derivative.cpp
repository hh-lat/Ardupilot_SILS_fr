
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

	v_rotation_matrices_update(vehcle.phi,  vehcle.theta,  vehcle.psi,  vehcle.alpha,  vehcle.beta);

	 //updates J, RPM, Cmu based on throttle inputs & V_inf
	 //updates vehcle.all_rotors_force[] and vehcle.all_rotors_moment[] in body frame
	v_rotor_dynamics(vehcle.step_dt);

	v_update_vehcle_Cmu();

	v_aero_force_and_moments();// updates vehcle.all_aero_force[] and vehcle.all_aero_moment[]

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
			vehcle.plane_on_ground = 0;
			vehcle.MLG_NR = (vehcle.mass*vehcle.g*vehcle.MLG_x + m)/(vehcle.FLG_x + vehcle.MLG_x);
			vehcle.FLG_NR = vehcle.mass*vehcle.g - vehcle.MLG_NR;

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

			m = 0;//m + vehcle.FLG_NR*vehcle.FLG_x - vehcle.MLG_NR*vehcle.MLG_x;

			dydt[0] = ((fx * mass_inv) + (r * v) - (q * w));//%u_dot//optimization
			dydt[1] = 0*((fy * mass_inv) + (p * w) - (r * u));//%v_dot
			dydt[2] = 0*((fz * mass_inv) + (q * u) - (p * v));  //%w_dot

			dydt[3] = 0*((Iyz * Iyz - Iy * Iz) * (l + q * (Ixz * p + Iyz * q - Iz * r) - r * (Ixy * p - Iy * q + Iyz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixz*Iy + Ixy*Iyz)*(n + p*(Ixy*p - Iy*q + Iyz*r) - q*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iyz + Ixy*Iz)*(m - p*(Ixz*p + Iyz*q - Iz*r) + r*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
			dydt[4] = 0*((Ixz * Ixz - Ix * Iz) * (m - p * (Ixz * p + Iyz * q - Iz * r) + r * (Ixy * q - Ix * p + Ixz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixy*Ixz + Ix*Iyz)*(n + p*(Ixy*p - Iy*q + Iyz*r) - q*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iyz + Ixy*Iz)*(l + q*(Ixz*p + Iyz*q - Iz*r) - r*(Ixy*p - Iy*q + Iyz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
			dydt[5] = 0*((Ixy * Ixy - Ix * Iy) * (n + p * (Ixy * p - Iy * q + Iyz * r) - q * (Ixy * q - Ix * p + Ixz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixy*Ixz + Ix*Iyz)*(m - p*(Ixz*p + Iyz*q - Iz*r) + r*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iy + Ixy*Iyz)*(l + q*(Ixz*p + Iyz*q - Iz*r) - r*(Ixy*p - Iy*q + Iyz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
			
			dydt[6] = 0*(p + (q * sinf(phi) * tanf(theta)) + (r * cosf(phi) * tanf(theta)));//%phi_dot
			dydt[7] = 0*((q * cosf(phi)) - (r * sinf(phi)));//%theta_dot
			dydt[8] = 0*((q * sinf(phi) * sec(theta)) + (r * cosf(phi) * sec(theta)));//%psi_dot

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
			vehcle.plane_on_ground = 0;

			float fz_mmg=0,fx_mmg=0;
			float A=0,B=0,C=0;

			A = vehcle.MLG_x*cosf(theta) - vehcle.MLG_z*sinf(theta);
			B = vehcle.MLG_x*sinf(theta) + vehcle.MLG_z*cosf(theta);

			C = 1 - (vehcle.mass*A*vehcle.MLG_x)/(Iy);
			if (fabsf(C)<0.0001)
			{
				C=0.0001; // to avoid division by zero
			}

			vehcle.MLG_NR = (vehcle.mass*vehcle.g + fz_mmg*cosf(theta) - fx_mmg*sinf(theta) +
							 vehcle.mass*((A*m/Iy) - B*q*q))/(C); 
			vehcle.FLG_NR = 0;

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

			fx = fx + vehcle.MLG_NR*sinf(theta);
			fz = fz - vehcle.MLG_NR*cosf(theta);

			m = m + vehcle.FLG_NR*vehcle.FLG_x - vehcle.MLG_NR*vehcle.MLG_x;

			dydt[0] = ((fx * mass_inv) + (r * v) - (q * w));//%u_dot//optimization
			dydt[1] = 0*((fy * mass_inv) + (p * w) - (r * u));//%v_dot
			dydt[2] = ((fz * mass_inv) + (q * u) - (p * v));  //%w_dot

			dydt[3] = 0*((Iyz * Iyz - Iy * Iz) * (l + q * (Ixz * p + Iyz * q - Iz * r) - r * (Ixy * p - Iy * q + Iyz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixz*Iy + Ixy*Iyz)*(n + p*(Ixy*p - Iy*q + Iyz*r) - q*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iyz + Ixy*Iz)*(m - p*(Ixz*p + Iyz*q - Iz*r) + r*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
			dydt[4] =  ((Ixz * Ixz - Ix * Iz) * (m - p * (Ixz * p + Iyz * q - Iz * r) + r * (Ixy * q - Ix * p + Ixz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixy*Ixz + Ix*Iyz)*(n + p*(Ixy*p - Iy*q + Iyz*r) - q*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iyz + Ixy*Iz)*(l + q*(Ixz*p + Iyz*q - Iz*r) - r*(Ixy*p - Iy*q + Iyz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);

			dydt[5] = 0*((Ixy * Ixy - Ix * Iy) * (n + p * (Ixy * p - Iy * q + Iyz * r) - q * (Ixy * q - Ix * p + Ixz * r)))/(Iz * Ixy * Ixy + 2 * Ixy * Ixz * Iyz + Iy * Ixz * Ixz + Ix * Iyz * Iyz - Ix*Iy*Iz) - ((Ixy*Ixz + Ix*Iyz)*(m - p*(Ixz*p + Iyz*q - Iz*r) + r*(Ixy*q - Ix*p + Ixz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz) - ((Ixz*Iy + Ixy*Iyz)*(l + q*(Ixz*p + Iyz*q - Iz*r) - r*(Ixy*p - Iy*q + Iyz*r)))/(Iz*Ixy*Ixy + 2*Ixy*Ixz*Iyz + Iy*Ixz*Ixz + Ix*Iyz*Iyz - Ix*Iy*Iz);
			
			dydt[6] = 0*(p + (q * sinf(phi) * tanf(theta)) + (r * cosf(phi) * tanf(theta)));//%phi_dot
			dydt[7] = ((q * cosf(phi)) - (r * sinf(phi)));//%theta_dot
			dydt[8] = 0*((q * sinf(phi) * sec(theta)) + (r * cosf(phi) * sec(theta)));//%psi_dot

			body_to_NED(vehcle.V_b_gnd,  V);

			dydt[9]  = V[0];//%inertial velocity X
			dydt[10] = V[1];//%inertial velocity Y
			dydt[11] = V[2];//%inertial velocity Z

			vehcle.total_force_bd[0] = fx;
			vehcle.total_force_bd[1] = 0;
			vehcle.total_force_bd[2] = fz;
			vehcle.total_moment_bd[0] = 0;
			vehcle.total_moment_bd[1] = m;
			vehcle.total_moment_bd[2] = 0;

			break;	
		}
		case IN_AIR:
		{
			vehcle.plane_on_ground = 0;

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

