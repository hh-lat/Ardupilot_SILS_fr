/*
 * Earth_model.c
 *      Author: rajat
 */


#include "stdio.h"

#include "math.h"

#include "stdlib.h"

#include "LAT_SIM_Earth_model.h"

/*
void main()
{
	float lla_pos[3] = {0.0};
	float lla_0[3] = {0.0};
	int flag_earth_model = 1;
	float nedPos[3] = {0.0};

	lla_pos[0] = 80.3076105196429f;  lla_pos[1] =  68.1891506375000f  ;  lla_pos[2] = 3600.0f;
	lla_0[0]   = 34.2968306696429f;  lla_0[1]   =  77.1872543875000f  ;  lla_0[2]   = 5400.0f;
	v_lla2ned(lla_pos, lla_0, flag_earth_model, nedPos);


	printf("%f nedPos[0]\n", nedPos[0]);
	printf("%f nedPos[1]\n", nedPos[1]);
	printf("%f nedPos[2]\n", nedPos[2]);


}*/

void v_lla2ned(float lla_pos[3], float lla_0[3], int flag_earth_model, float nedPos[3])
{

	float ecef_pos[3] = {0.0};
	float ecef_0[3] = {0.0};

	switch (flag_earth_model)
	{
	case 0:
	{
		v_lla2ecef( lla_pos,  ecef_pos); /*ELLIPSOID*/
		v_lla2ecef( lla_0,  ecef_0);

		float ecefPosWithENUOrigin[3] = {0.0f};
		ecefPosWithENUOrigin[0] = ecef_pos[0] - ecef_0[0];
		ecefPosWithENUOrigin[1] = ecef_pos[1] - ecef_0[1];
		ecefPosWithENUOrigin[2] = ecef_pos[2] - ecef_0[2];

		float x, y, z;
		x = ecefPosWithENUOrigin[0];
		y = ecefPosWithENUOrigin[1];
		z = ecefPosWithENUOrigin[2];

		float phi = lla_0[0];
		float lambda = lla_0[1];
		float tmp, uEast, vNorth, wUp;

		uEast  = -sin(lambda / 57.2957795130823f)  * x   + cos(lambda / 57.2957795130823f) * y;
		tmp    =  cos(lambda / 57.2957795130823f)  * x   + sin(lambda / 57.2957795130823f) * y;
		vNorth = -sin(phi / 57.2957795130823f)     * tmp + cos(phi / 57.2957795130823f)   * z;
		wUp    =  cos(phi / 57.2957795130823f)     * tmp + sin(phi / 57.2957795130823f)   * z;

		nedPos[0] = vNorth;
		nedPos[1] = uEast;
		nedPos[2] = -wUp;

		break;
	}


	case 1:
	{
		float f1  = 0.00335281066474748f; /*FLAT*/
		/**equatorial radius*/
		float R =  6378137.0f;

		float dLat = lla_pos[0] - lla_0[0];
		float dLon = lla_pos[1] - lla_0[1];



		float Rn = R / sqrtf(1.0f - (2.0f * f1 - f1 * f1) * sin(lla_0[0] / 57.2957795130823f) * sin(lla_0[0] / 57.2957795130823f));


		float Rm = Rn * ((1.0f - (2.0f * f1 - f1 * f1)) / (1.0f - (2.0f * f1 - f1 * f1) * sin(lla_0[0] / 57.2957795130823f) * sin(lla_0[0] / 57.2957795130823f)));

		float dNorth = dLat / ((57.2957795130823f)*atanf(1.0f / Rm));
		float dEast = dLon / ((57.2957795130823f)*atanf(1.0f / (Rn * cos(lla_0[0] / 57.2957795130823f))));

		nedPos[0] = dNorth;
		nedPos[1] = dEast;
		nedPos[2] = -lla_pos[2] + lla_0[2];


		break;
	}
	}



}




void v_lla2ecef(float lla_pos[3], float ecef_pos[3])
{
	float a = 6378137.0f; /*semi major axis*/
	float f = 1.0f/ 298.257223563; /*flattening*/
	//float mu = 398600500000000.0f;/*Gravitational Constant(m3/s2)*/
	//float w = 0.000072921151467f;/*Angular speed of earth*/
	/*geodetic to cylindrical*/
	float sinphi = sin(lla_pos[0] / 57.2957795130823f);
	float cosphi = cos(lla_pos[0] / 57.2957795130823f);


	float e2;
	float N;
	float rho;
	float z;

	float h = lla_pos[2];


	e2 = f * (2.0f - f);
	N  = a / sqrtf(1.0f - e2 * sinphi * sinphi);
	rho = (N + h) * cosphi;
	z = (N * (1.0f - e2) + h) * sinphi;


	ecef_pos[0] = rho * cos(lla_pos[1] / 57.2957795130823f);
	ecef_pos[1] = rho * sin(lla_pos[1] / 57.2957795130823f);
	ecef_pos[2] = z;
}





