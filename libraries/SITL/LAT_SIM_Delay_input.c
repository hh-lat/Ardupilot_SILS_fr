#include <math.h>
#include "math_util.h"
#include "plant.h"


float Delay_input_fn_T(float new_value_T)
{
	float temp_old_T = delay_array_T[delay_array_length-1];

	for (int id = delay_array_length-1 ; id > 0;  id = id-1)
	{
		delay_array_T[id] = delay_array_T[id-1];
	}

	delay_array_T[0] = new_value_T;

	return temp_old_T;
}

// used to input transport delay in delay_array_T1 array
// the delay will be given by delay_array_length
float Delay_input_fn_T1(float new_value_T)
{
	float temp_old_T = delay_array_T1[delay_array_length-1];

	for (int id = delay_array_length-1; id > 0 ; id = id-1)
	{
		delay_array_T1[id] = delay_array_T1[id-1];
	}
	delay_array_T1[0] = new_value_T;

	return temp_old_T;
}


void pure_transport_delay_int(uint16_t out[],uint16_t new[],uint16_t bufferarray[][quad_num_motors + fwv_motors + num_actuator],int len, int delay_array_length)
{

	memcpy(out, bufferarray[delay_array_length-1], len * sizeof(uint16_t) );

	for (int id = delay_array_length-1; id > 0; id = id-1)
	{
		memmove(bufferarray[id],bufferarray[id-1],len * sizeof(uint16_t) );
	}

	memmove(bufferarray[0],new,len * sizeof(uint16_t));
}

void pure_transport_delay_float(float new[],float bufferarray[][quad_num_motors + fwv_motors + num_actuator],int len, int delay_array_length)
{
	float arrtemp[len];

	memcpy(arrtemp, bufferarray[delay_array_length-1], len * sizeof(float) );
	for (int id = delay_array_length-1; id > 0; id = id-1)
	{
		memmove(bufferarray[id],bufferarray[id-1],len * sizeof(float) );
	}
	memmove(bufferarray[0],new,len * sizeof(float));

	new = arrtemp;
}

