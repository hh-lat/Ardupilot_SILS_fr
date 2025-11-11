
#include "Payload_force_and_moment.h"
#include <time.h>

static int payload_fandm_adder_divider =0;
static int loop_payload=0;


void Payload_forces_and_moments_body_frame()
{

	payload_fandm_adder_divider = 1000; // put here 1000 if you want to update payload foce at 1 hz
                                   // basically put here = plant hz/ payload hz, so if you want payload hz=2, then put here 1000/2 = 500
	if((loop_payload % payload_fandm_adder_divider) == 0)
	{
		Payload_fire_fighting_tube();
	}
	loop_payload = loop_payload + 1;

	if (loop_payload >= 1001)
	{
		loop_payload=1;
	}
}



void Payload_fire_fighting_tube()
{

	float pipe_radius = 0.014;
	float area = 0;
	float pressure = 0.0;//21.4 kgf/cm2, peak pressure 35.7 kgf/cm2
	float force[3] ={0.0};
	float cg_offset_force[3]={0.0f};
	int pp,qq,rr,ss,tt,uu,vv,ww,xx;
	//https://www.msfiresafetyservices.com/fire-hose-pipe/

	srand(loop_payload + time(NULL) );
	pp=rand()%10000;

	cg_offset_force[0] = powf(-1.0,pp)*11.0/100.0f;
	cg_offset_force[1] = powf(-1.0,qq)*10.0/100.0f;
	cg_offset_force[2] = powf(-1.0,rr)*30.0f/100.0f;

	area = 3.1415f*pipe_radius*pipe_radius;

	force[0] = 0.0f;
	force[1] = 0.0f;
	force[2] = 0.0f;

	vehicle.all_payload_force[0] = powf(-1.0,vv)*force[0] + (powf(-1.0,ss)*force[0]/10.0f);
	vehicle.all_payload_force[1] = powf(-1.0,ww)*force[1] + (powf(-1.0,tt)*force[1]/10.0f);
	vehicle.all_payload_force[2] = powf(-1.0,xx)*force[2] + (powf(-1.0,uu)*force[2]/10.0f);

	vehicle.all_payload_moment[0] =  vehicle.all_payload_force[2]*cg_offset_force[1] - vehicle.all_payload_force[1]*cg_offset_force[2];
	vehicle.all_payload_moment[1] = -vehicle.all_payload_force[2]*cg_offset_force[0] + vehicle.all_payload_force[0]*cg_offset_force[2];
	vehicle.all_payload_moment[2] =  vehicle.all_payload_force[1]*cg_offset_force[0] - vehicle.all_payload_force[0]*cg_offset_force[1];
}

