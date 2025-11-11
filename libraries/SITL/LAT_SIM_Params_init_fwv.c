#include "plant.h"
#include "common_variable.h"

void v_param_init_fwv()
{
	g    = 9.80865;		// m/sec^2
	c    = 0.2104;      // mean aerodynamic chord
	mass = 6.0;     	// mass in kgs
	e    = 0.9;      	// ostwald efficiency factor
	s    = 0.4626;		// reference area in m^2
	b    = 2.2;			// full span of wing in m

	// s = length *breadth * percentage of this top surface area that has surface and not holes * sin(tilt angle)
	Ix = 0.6275, Iy = 0.564, Iz = 1.176, Ixy = 0.0, Ixz = 0.0, Iyz = 0.0;  // kgm^2
/************************************************************************/


/*
	//% thrust factor
	b1 = 0.10732;
	//% torque factor
	d1 = 0.00475;
	*/
	
	b1 = 0.000075823f;
	b1_fwv = 0.000075823f;
	//% torque factor
	d1 = 0.000001513f;
	d1_fwv = 0.000001513f;

	//define aerodynamic coefficients related to aero angles
    vehicle.CLo = 0.262f; vehicle.CL_alpha = 5.33f; vehicle.CL_q = 6.027f;
    vehicle.CDo = 0.0446f; vehicle.CD_alpha = 0.369f;
    vehicle.CYo = 0.0f; vehicle.CY_beta = -0.23f; vehicle.CY_p = 0.0f; vehicle.CY_r = 0.128f;

    vehicle.Clo = 0.0f; vehicle.Cl_beta = -0.057f;  vehicle.Cl_p = -0.567f; vehicle.Cl_r = 0.203f;
    vehicle.Cmo = 0.064f; vehicle.Cm_alpha = -1.2f; vehicle.Cm_q = -16.84f;
    vehicle.Cno = 0.0f; vehicle.Cn_beta = 0.047f;  vehicle.Cn_p = -0.084f; vehicle.Cn_r = -0.054f;
}
