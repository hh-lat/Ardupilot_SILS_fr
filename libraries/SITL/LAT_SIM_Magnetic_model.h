/* Rajat*/

#include <stdio.h>
#include <stdint.h>

typedef struct
{
	float F;
	float I;
	float X;
	float Y;
	float Z;
	float H;
	float D;

	uint8_t data_health;

}Struct_magneticmodel;

extern Struct_magneticmodel s_earth_mm;

extern void v_magnetic_model_run(float , float , float , float , float , float );
extern void v_magnetic_model_init();
