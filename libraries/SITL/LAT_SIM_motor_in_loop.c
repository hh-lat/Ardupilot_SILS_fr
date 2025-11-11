#include <plant.h>

static float mil_rpm_before_noise[quad_num_motors + fwv_motors] = {-500};
static int count_motor_noise[quad_num_motors + fwv_motors]      = {0};
static int noise_remover_index        = 24;
void motor_in_loop_thrust_genrtr()
{

	for(int f = 0;f < 4; f++)
	{

		if(((gnc_rpm_in[f]- mil_rpm[f]) > 400)  && (count_motor_noise[f] < noise_remover_index) )
		{
		    count_motor_noise[f]= count_motor_noise[f] + 1;

		/*	if ( (fabsf(pwm_in[f]- mil_rpm_before_noise[f]) < 400.0) )
			{
   				mil_rpm[f]=pwm_in[f];
   				mil_rpm_before_noise[f]=pwm_in[f];
   				count_motor_noise[f]=0;
			}*/

		}
		else
		{
		/*	if( count_motor_noise[f] > noise_remover_index )
			{
				mil_rpm_before_noise[f]=mil_rpm[f];
			}*/

		  mil_rpm[f] = gnc_rpm_in[f];
		  count_motor_noise[f] = 0;
		}
		rotor_force_out[f]= -b1 * (mil_rpm[f] * mil_rpm[f]) * (2.0f * 3.1415f * 2.0 * 3.1415f/3600.0f);
	}
//    rotor_force_out[0]= -b1*(mil_rpm[4]*mil_rpm[4])*(2.0f*3.1415f*2.0*3.1415f/3600.0f);
//    rotor_force_out[1]= -b1*(mil_rpm[5]*mil_rpm[5])*(2.0f*3.1415f*2.0*3.1415f/3600.0f);
//    rotor_force_out[2]= -b1*(mil_rpm[6]*mil_rpm[6])*(2.0f*3.1415f*2.0*3.1415f/3600.0f);
//    rotor_force_out[3]= -b1*(mil_rpm[7]*mil_rpm[7])*(2.0f*3.1415f*2.0*3.1415f/3600.0f);

}
