#include <math.h>
#include "LAT_SIM_math_util.h"
#include "LAT_SIM_Runner.h"
#include "LAT_SIM_rk4.h"
#include "LAT_SIM_derivative.h"



void v_rk4(float y[],float t,float h)
{
	float VX=0,VY=0,VZ=0;
	float AX=0,AY=0,AZ=0;
	float phi=0, theta=0, psi=0;

	float h2 = 0.0, h6 = 0.0, th = 0.0;

	float k1[NUM_STATEVARS], k2[NUM_STATEVARS], k3[NUM_STATEVARS], k4[NUM_STATEVARS]; // differential value of y wrt t
	float yt[NUM_STATEVARS];	
	int i = 0;

	array_initd(yt, NUM_STATEVARS);
	array_initd(k1, NUM_STATEVARS);
	array_initd(k2, NUM_STATEVARS);
	array_initd(k3, NUM_STATEVARS);
	array_initd(k4, NUM_STATEVARS);

	h2 = h/2;
	h6 = h/6;

	v_derivative(y, t, k1);// supply y to get k1


	for(i = 0; i < NUM_STATEVARS; i++)
	{
		yt[i] = y[i] + (h2 * k1[i]); // yt is y + (h/2) *k1
	}

	th = t + h2;
	v_derivative(yt, th, k2); // now supply yt to get k2


	for(i = 0; i < NUM_STATEVARS; i++)
	{
		yt[i] = y[i] + (h2 * k2[i]); // yt is y + (h/2) *k2
	}

	v_derivative(yt, th, k3);// now supply yt to get k3

	for(i = 0; i < NUM_STATEVARS; i++)
	{
		yt[i] = y[i]+ h * k3[i];// here yt is y + h*k3
	}

	v_derivative(yt, t+h, k4); // now supply yt to get k4


	switch(vehicle.dof)
	{
	case DOF_ALL_MOTION:
	{
		if (s_grnd_model.plane_on_ground == 1)
		{
			y[0] = 0.0;   // u
			y[1] = 0.0;   // v
			y[2] = 0.0;   // w
			y[3] = 0.0;   // p
			y[4] = 0.0;   // q
			y[5] = 0.0;   // r
			y[6] = y[6];  // phi
			y[7] = y[7];  // theta
			y[8] = y[8];  // psi
			y[9] = y[9];  // x
			y[10]= y[10]; // y
			y[11]= y[11]; // z

			// send true accelerations, which is force/mass , where force is total force including gravity
			acc_real_plant[0] = (0.0);//
			acc_real_plant[1] = (0.0);// true acceleration in m/sec^2 , sent to Pseudo Ins, 26 may 2021
			acc_real_plant[2] = (0.0);
		}
		else // Plane in air
		{
			// Adding all k1, k2, k3, k4 with weights
			for(i = 0; i < NUM_STATEVARS; i++)
			{
				y[i] = y[i] + h6 * (k1[i] + k4[i] + 2.0f*(k2[i] + k3[i])); // outputs y(t+h) = y(t) + h/6*(k1 + k4 + 2*(k1+k2) )
			}
		}
		break;
	}

	case DOF_ROLL:
	{
		y[0] = y[0] + h6 * (k1[0] + k4[0] + 2.0f*(k2[0] + k3[0]))*0;      //u
		y[1] = y[1] + h6 * (k1[1] + k4[1] + 2.0f*(k2[1] + k3[1]))*0;      //v
		y[2] = y[2] + h6 * (k1[2] + k4[2] + 2.0f*(k2[2] + k3[2]))*0;      //w
		y[3] = y[3] + h6 * (k1[3] + k4[3] + 2.0f*(k2[3] + k3[3]));        //p
		y[4] = y[4] + h6 * (k1[4] + k4[4] + 2.0f*(k2[4] + k3[4]))*0;      //q
		y[5] = y[5] + h6 * (k1[5] + k4[5] + 2.0f*(k2[5] + k3[5]))*0;      //r
		y[6] = y[6] + h6 * (k1[6] + k4[6] + 2.0f*(k2[6] + k3[6]));        //phi
		y[7] = y[7] + h6 * (k1[7] + k4[7] + 2.0f*(k2[7] + k3[7]))*0;      //theta
		y[8] = y[8] + h6 * (k1[8] + k4[8] + 2.0f*(k2[8] + k3[8]))*0;      //psi
		y[9] = y[9] + h6 * (k1[9] + k4[9] + 2.0f*(k2[9] + k3[9]))*0;      //x
		y[10] = y[10] + h6 * (k1[10] + k4[10] + 2.0f*(k2[10] + k3[10]))*0;//y
		y[11] = y[11] + h6 * (k1[11] + k4[11] + 2.0f*(k2[11] + k3[11]))*0;//z

		// send true accelerations, which is force/mass , where force is total force including gravity
		acc_real_plant[0] = (0.0);//
		acc_real_plant[1] = (0.0);
		acc_real_plant[2] = (0.0);

		break;
	}

	case DOF_PITCH:
	{
		y[0] = y[0] + h6 * (k1[0] + k4[0] + 2.0f*(k2[0] + k3[0]))*0;      //u
		y[1] = y[1] + h6 * (k1[1] + k4[1] + 2.0f*(k2[1] + k3[1]))*0;      //v
		y[2] = y[2] + h6 * (k1[2] + k4[2] + 2.0f*(k2[2] + k3[2]))*0;      //w
		y[3] = y[3] + h6 * (k1[3] + k4[3] + 2.0f*(k2[3] + k3[3]))*0;      //p
		y[4] = y[4] + h6 * (k1[4] + k4[4] + 2.0f*(k2[4] + k3[4]));        //q
		y[5] = y[5] + h6 * (k1[5] + k4[5] + 2.0f*(k2[5] + k3[5]))*0;      //r
		y[6] = y[6] + h6 * (k1[6] + k4[6] + 2.0f*(k2[6] + k3[6]))*0;      //phi
		y[7] = y[7] + h6 * (k1[7] + k4[7] + 2.0f*(k2[7] + k3[7]));        //theta
		y[8] = y[8] + h6 * (k1[8] + k4[8] + 2.0f*(k2[8] + k3[8]))*0;      //psi
		y[9] = y[9] + h6 * (k1[9] + k4[9] + 2.0f*(k2[9] + k3[9]))*0;      //x
		y[10] = y[10] + h6 * (k1[10] + k4[10] + 2.0f*(k2[10] + k3[10]))*0;//y
		y[11] = y[11] + h6 * (k1[11] + k4[11] + 2.0f*(k2[11] + k3[11]))*0;//z

		// send true accelerations, which is force/mass , where force is total force including gravity
		acc_real_plant[0] = (0.0);//
		acc_real_plant[1] = (0.0);
		acc_real_plant[2] = (0.0);
		break;
	}

	case DOF_YAW:
	{
		y[0] = y[0] + h6 * (k1[0] + k4[0] + 2.0f*(k2[0] + k3[0]))*0;      //u
		y[1] = y[1] + h6 * (k1[1] + k4[1] + 2.0f*(k2[1] + k3[1]))*0;      //v
		y[2] = y[2] + h6 * (k1[2] + k4[2] + 2.0f*(k2[2] + k3[2]))*0;      //w
		y[3] = y[3] + h6 * (k1[3] + k4[3] + 2.0f*(k2[3] + k3[3]))*0;      //p
		y[4] = y[4] + h6 * (k1[4] + k4[4] + 2.0f*(k2[4] + k3[4]))*0;      //q
		y[5] = y[5] + h6 * (k1[5] + k4[5] + 2.0f*(k2[5] + k3[5]));        //r
		y[6] = y[6] + h6 * (k1[6] + k4[6] + 2.0f*(k2[6] + k3[6]))*0;      //phi
		y[7] = y[7] + h6 * (k1[7] + k4[7] + 2.0f*(k2[7] + k3[7]))*0;      //theta
		y[8] = y[8] + h6 * (k1[8] + k4[8] + 2.0f*(k2[8] + k3[8]));        //psi
		y[9] = y[9] + h6 * (k1[9] + k4[9] + 2.0f*(k2[9] + k3[9]))*0;      //x
		y[10] = y[10] + h6 * (k1[10] + k4[10] + 2.0f*(k2[10] + k3[10]))*0;//y
		y[11] = y[11] + h6 * (k1[11] + k4[11] + 2.0f*(k2[11] + k3[11]))*0;//z

		// send true accelerations, which is force/mass , where force is total force including gravity
		acc_real_plant[0] = (0.0);//
		acc_real_plant[1] = (0.0);
		acc_real_plant[2] = (0.0);
		break;
	}

	case DOF_ROLL_PITCH:
	{
		y[0] = y[0] + h6 * (k1[0] + k4[0] + 2.0f*(k2[0] + k3[0]))*0;      //u
		y[1] = y[1] + h6 * (k1[1] + k4[1] + 2.0f*(k2[1] + k3[1]))*0;      //v
		y[2] = y[2] + h6 * (k1[2] + k4[2] + 2.0f*(k2[2] + k3[2]))*0;      //w
		y[3] = y[3] + h6 * (k1[3] + k4[3] + 2.0f*(k2[3] + k3[3]));      //p
		y[4] = y[4] + h6 * (k1[4] + k4[4] + 2.0f*(k2[4] + k3[4]));      //q
		y[5] = y[5] + h6 * (k1[5] + k4[5] + 2.0f*(k2[5] + k3[5]))*0;      //r
		y[6] = y[6] + h6 * (k1[6] + k4[6] + 2.0f*(k2[6] + k3[6]));      //phi
		y[7] = y[7] + h6 * (k1[7] + k4[7] + 2.0f*(k2[7] + k3[7]));      //theta
		y[8] = y[8] + h6 * (k1[8] + k4[8] + 2.0f*(k2[8] + k3[8]))*0;      //psi
		y[9] = y[9] + h6 * (k1[9] + k4[9] + 2.0f*(k2[9] + k3[9]))*0;      //x
		y[10] = y[10] + h6 * (k1[10] + k4[10] + 2.0f*(k2[10] + k3[10]))*0;//y
		y[11] = y[11] + h6 * (k1[11] + k4[11] + 2.0f*(k2[11] + k3[11]))*0;//z

		// send true accelerations, which is force/mass , where force is total force including gravity
		acc_real_plant[0] = (0.0);//
		acc_real_plant[1] = (0.0);
		acc_real_plant[2] = (0.0);
		break;
	}

	case DOF_ROLL_YAW:
	{
		y[0] = y[0] + h6 * (k1[0] + k4[0] + 2.0f*(k2[0] + k3[0]))*0;      //u
		y[1] = y[1] + h6 * (k1[1] + k4[1] + 2.0f*(k2[1] + k3[1]))*0;      //v
		y[2] = y[2] + h6 * (k1[2] + k4[2] + 2.0f*(k2[2] + k3[2]))*0;      //w
		y[3] = y[3] + h6 * (k1[3] + k4[3] + 2.0f*(k2[3] + k3[3]));        //p
		y[4] = y[4] + h6 * (k1[4] + k4[4] + 2.0f*(k2[4] + k3[4]))*0;      //q
		y[5] = y[5] + h6 * (k1[5] + k4[5] + 2.0f*(k2[5] + k3[5]));        //r
		y[6] = y[6] + h6 * (k1[6] + k4[6] + 2.0f*(k2[6] + k3[6]));        //phi
		y[7] = y[7] + h6 * (k1[7] + k4[7] + 2.0f*(k2[7] + k3[7]))*0;      //theta
		y[8] = y[8] + h6 * (k1[8] + k4[8] + 2.0f*(k2[8] + k3[8]));        //psi
		y[9] = y[9] + h6 * (k1[9] + k4[9] + 2.0f*(k2[9] + k3[9]))*0;      //x
		y[10] = y[10] + h6 * (k1[10] + k4[10] + 2.0f*(k2[10] + k3[10]))*0;//y
		y[11] = y[11] + h6 * (k1[11] + k4[11] + 2.0f*(k2[11] + k3[11]))*0;//z

		// send true accelerations, which is force/mass , where force is total force including gravity
		acc_real_plant[0] = (0.0);//
		acc_real_plant[1] = (0.0);
		acc_real_plant[2] = (0.0);
		break;
	}


	case DOF_PITCH_YAW:
	{
		y[0] = y[0] + h6 * (k1[0] + k4[0] + 2.0f*(k2[0] + k3[0]))*0;      //u
		y[1] = y[1] + h6 * (k1[1] + k4[1] + 2.0f*(k2[1] + k3[1]))*0;      //v
		y[2] = y[2] + h6 * (k1[2] + k4[2] + 2.0f*(k2[2] + k3[2]))*0;      //w
		y[3] = y[3] + h6 * (k1[3] + k4[3] + 2.0f*(k2[3] + k3[3]))*0;      //p
		y[4] = y[4] + h6 * (k1[4] + k4[4] + 2.0f*(k2[4] + k3[4]));      //q
		y[5] = y[5] + h6 * (k1[5] + k4[5] + 2.0f*(k2[5] + k3[5]));      //r
		y[6] = y[6] + h6 * (k1[6] + k4[6] + 2.0f*(k2[6] + k3[6]))*0;      //phi
		y[7] = y[7] + h6 * (k1[7] + k4[7] + 2.0f*(k2[7] + k3[7]));      //theta
		y[8] = y[8] + h6 * (k1[8] + k4[8] + 2.0f*(k2[8] + k3[8]));      //psi
		y[9] = y[9] + h6 * (k1[9] + k4[9] + 2.0f*(k2[9] + k3[9]))*0;      //x
		y[10] = y[10] + h6 * (k1[10] + k4[10] + 2.0f*(k2[10] + k3[10]))*0;//y
		y[11] = y[11] + h6 * (k1[11] + k4[11] + 2.0f*(k2[11] + k3[11]))*0;//z

		// send true accelerations, which is force/mass , where force is total force including gravity
		acc_real_plant[0] = (0.0);//
		acc_real_plant[1] = (0.0);
		acc_real_plant[2] = (0.0);
		break;
	}

	case DOF_ROLL_PITCH_YAW:
	{
		y[0] = y[0] + h6 * (k1[0] + k4[0] + 2.0f*(k2[0] + k3[0]))*0;      //u
		y[1] = y[1] + h6 * (k1[1] + k4[1] + 2.0f*(k2[1] + k3[1]))*0;      //v
		y[2] = y[2] + h6 * (k1[2] + k4[2] + 2.0f*(k2[2] + k3[2]))*0;      //w
		y[3] = y[3] + h6 * (k1[3] + k4[3] + 2.0f*(k2[3] + k3[3]));      //p
		y[4] = y[4] + h6 * (k1[4] + k4[4] + 2.0f*(k2[4] + k3[4]));      //q
		y[5] = y[5] + h6 * (k1[5] + k4[5] + 2.0f*(k2[5] + k3[5]));      //r
		y[6] = y[6] + h6 * (k1[6] + k4[6] + 2.0f*(k2[6] + k3[6]));      //phi
		y[7] = y[7] + h6 * (k1[7] + k4[7] + 2.0f*(k2[7] + k3[7]));      //theta
		y[8] = y[8] + h6 * (k1[8] + k4[8] + 2.0f*(k2[8] + k3[8]));      //psi
		y[9] = y[9] + h6 * (k1[9] + k4[9] + 2.0f*(k2[9] + k3[9]))*0;      //x
		y[10] = y[10] + h6 * (k1[10] + k4[10] + 2.0f*(k2[10] + k3[10]))*0;//y
		y[11] = y[11] + h6 * (k1[11] + k4[11] + 2.0f*(k2[11] + k3[11]))*0;//z

		// send true accelerations, which is force/mass , where force is total force including gravity
		acc_real_plant[0] = (0.0);//
		acc_real_plant[1] = (0.0);
		acc_real_plant[2] = (0.0);
		break;
	}

	case DOF_HORIZONTAL_MOTION:
	{
		y[0] = y[0] + h6 * (k1[0] + k4[0] + 2.0f*(k2[0] + k3[0]));        //u
		y[1] = y[1] + h6 * (k1[1] + k4[1] + 2.0f*(k2[1] + k3[1]));        //v
		y[2] = y[2] + h6 * (k1[2] + k4[2] + 2.0f*(k2[2] + k3[2]));        //w
		y[3] = y[3] + h6 * (k1[3] + k4[3] + 2.0f*(k2[3] + k3[3]));        //p
		y[4] = y[4] + h6 * (k1[4] + k4[4] + 2.0f*(k2[4] + k3[4]));        //q
		y[5] = y[5] + h6 * (k1[5] + k4[5] + 2.0f*(k2[5] + k3[5]));        //r
		y[6] = y[6] + h6 * (k1[6] + k4[6] + 2.0f*(k2[6] + k3[6]));        //phi
		y[7] = y[7] + h6 * (k1[7] + k4[7] + 2.0f*(k2[7] + k3[7]));        //theta
		y[8] = y[8] + h6 * (k1[8] + k4[8] + 2.0f*(k2[8] + k3[8]));        //psi
		y[9] = y[9] + h6 * (k1[9] + k4[9] + 2.0f*(k2[9] + k3[9]));        //x
		y[10] = y[10] + h6 * (k1[10] + k4[10] + 2.0f*(k2[10] + k3[10]));  //y
		y[11] = y[11] + h6 * (k1[11] + k4[11] + 2.0f*(k2[11] + k3[11]))*0;//z

		phi   = y[6];
		theta = y[7];
		psi   = y[8];

		// converting to NED frame and making Z velocity zero
		VX = y[2]*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - y[1]*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + cos(psi)*cos(theta)*y[0];
		VY = y[1]*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - y[2]*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + cos(theta)*sin(psi)*y[0];
		VZ = 0;

		// converting to body frame again with VZ =0
		y[0] = VX*cos(psi)*cos(theta) - VZ*sin(theta) + VY*cos(theta)*sin(psi);
		y[1] = VY*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - VX*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + VZ*cos(theta)*sin(phi);
		y[2] = VX*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - VY*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + VZ*cos(phi)*cos(theta);


		// converting to NED frame and making Z true acceleration zero
		AX = acc_real_plant[2]*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - acc_real_plant[1]*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + cos(psi)*cos(theta)*acc_real_plant[0];
		AY = acc_real_plant[1]*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - acc_real_plant[2]*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + cos(theta)*sin(psi)*acc_real_plant[0];
		AZ = 0;

		// converting to body frame again with AZ =0
		acc_real_plant[0] = AX*cos(psi)*cos(theta) - AZ*sin(theta) + AY*cos(theta)*sin(psi);
		acc_real_plant[1] = AY*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - AX*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + AZ*cos(theta)*sin(phi);
		acc_real_plant[2] = AX*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - AY*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + AZ*cos(phi)*cos(theta);
		break;
	}

	case DOF_VERTICAL_MOTION:
	{
		y[0] = y[0] + h6 * (k1[0] + k4[0] + 2.0f*(k2[0] + k3[0]));        //u
		y[1] = y[1] + h6 * (k1[1] + k4[1] + 2.0f*(k2[1] + k3[1]));        //v
		y[2] = y[2] + h6 * (k1[2] + k4[2] + 2.0f*(k2[2] + k3[2]));        //w
		y[3] = y[3] + h6 * (k1[3] + k4[3] + 2.0f*(k2[3] + k3[3]));        //p
		y[4] = y[4] + h6 * (k1[4] + k4[4] + 2.0f*(k2[4] + k3[4]));        //q
		y[5] = y[5] + h6 * (k1[5] + k4[5] + 2.0f*(k2[5] + k3[5]));        //r
		y[6] = y[6] + h6 * (k1[6] + k4[6] + 2.0f*(k2[6] + k3[6]));        //phi
		y[7] = y[7] + h6 * (k1[7] + k4[7] + 2.0f*(k2[7] + k3[7]));        //theta
		y[8] = y[8] + h6 * (k1[8] + k4[8] + 2.0f*(k2[8] + k3[8]));        //psi
		y[9] = y[9] + h6 * (k1[9] + k4[9] + 2.0f*(k2[9] + k3[9]))*0;      //x
		y[10] = y[10] + h6 * (k1[10] + k4[10] + 2.0f*(k2[10] + k3[10]))*0;//y
		y[11] = y[11] + h6 * (k1[11] + k4[11] + 2.0f*(k2[11] + k3[11]));  //z

		phi   = y[6];
		theta = y[7];
		psi   = y[8];

		// converting to NED frame and making X,Y velocity zero
		VX = 0;//y[2]*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - y[1]*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + cos(psi)*cos(theta)*y[0];
		VY = 0;//y[1]*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - y[2]*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + cos(theta)*sin(psi)*y[0];
		VZ = cos(phi)*cos(theta)*y[2] - sin(theta)*y[0] + cos(theta)*sin(phi)*y[1];

		// converting to body frame again with VX,VY =0
		y[0] = VX*cos(psi)*cos(theta) - VZ*sin(theta) + VY*cos(theta)*sin(psi);
		y[1] = VY*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - VX*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + VZ*cos(theta)*sin(phi);
		y[2] = VX*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - VY*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + VZ*cos(phi)*cos(theta);

		// converting to NED frame and making X,y acc_real_plant acceleration zero
		AX = 0;//acc_real_plant[2]*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - acc_real_plant[1]*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + cos(psi)*cos(theta)*acc_real_plant[0];
		AY = 0;//acc_real_plant[1]*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - acc_real_plant[2]*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + cos(theta)*sin(psi)*acc_real_plant[0];
		AZ = cos(phi)*cos(theta)*acc_real_plant[2] - sin(theta)*acc_real_plant[0] + cos(theta)*sin(phi)*acc_real_plant[1];

		// converting to body frame again with AX,AY =0
		acc_real_plant[0] = AX*cos(psi)*cos(theta) - AZ*sin(theta) + AY*cos(theta)*sin(psi);
		acc_real_plant[1] = AY*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - AX*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + AZ*cos(theta)*sin(phi);
		acc_real_plant[2] = AX*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - AY*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + AZ*cos(phi)*cos(theta);

		break;
	}
	}


}


