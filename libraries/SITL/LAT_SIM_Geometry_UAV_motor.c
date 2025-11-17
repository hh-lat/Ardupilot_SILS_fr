#include "LAT_SIM_Runner.h"
#include "LAT_SIM_Forces_and_moments_rotors.h"
#include "LAT_SIM_Forces_and_moments_ctrl_srfce.h"
#include "LAT_SIM_Geometry_UAV_motor.h"




void Configure_rotor_geometry()
{

#ifdef Quadplane
	//1,2,3,4 are upward motors, 5 is pusher motor
	/*
		  1cw   5ccw   2ccw
	      ##     ##     ##
	      ##     ##     ##
	      ##     ##     ##
	############################
	############################
	############################
	      ##     ##     ##
	      ##   ######   ##
	      ##   ######   ##
	     4ccw           3cw
	*/
	int i=1;

	float x =1.0, y=1.0;

	/*distance from cg to rotor reference line in x,y axis*/
     x = 0.47f;
     y=  0.31f;

     /*x,y,z positions of all motors (cg at 0,0,0)*/
	 rotor_xyz[1-i][0]= x;			rotor_xyz[5-i][0]= x;			rotor_xyz[2-i][0]= x;
	 rotor_xyz[1-i][1]=-y;          rotor_xyz[5-i][1]= 0.0;         rotor_xyz[2-i][1]= y;
	 rotor_xyz[1-i][2]= 0.0;        rotor_xyz[5-i][2]= 0.0;	        rotor_xyz[2-i][2]= 0.0;

	 rotor_xyz[4-i][0]=-x;                                          rotor_xyz[3-i][0]=-x;
	 rotor_xyz[4-i][1]=-y;                                          rotor_xyz[3-i][1]= y;
	 rotor_xyz[4-i][2]= 0.0;                                        rotor_xyz[3-i][2]= 0.0;

	 /*rotor tilt angle (upwards as reference and cw is +ve)*/
	 rotor_tilt[1-i][0]= 0.0;		rotor_tilt[5-i][0]= 0.0;                rotor_tilt[2-i][0]= 0.0;
	 rotor_tilt[1-i][1]= 0.0;       rotor_tilt[5-i][1]=-90.0f/57.2958f;		rotor_tilt[2-i][1]= 0.0;
	 rotor_tilt[1-i][2]= 0.0;       rotor_tilt[5-i][2]= 0.0;                rotor_tilt[2-i][2]= 0.0;

	 rotor_tilt[4-i][0]=0.0;                      							rotor_tilt[3-i][0]=0.0;
	 rotor_tilt[4-i][1]=0.0;                      							rotor_tilt[3-i][1]=0.0;
	 rotor_tilt[4-i][2]=-180.0f/57.2958f;         							rotor_tilt[3-i][2]=180.0f/57.2958f;

	 /*torque produced due to rotor rotation*/
	 rotor_r_direction[1-i]=-1.0;	rotor_r_direction[5-i]= 0;/*should not be zero for giving roll torque*/		rotor_r_direction[2-i]= 1.0;
	 rotor_r_direction[4-i]= 1.0;                 																rotor_r_direction[3-i]=-1.0;


     // SIGN CONVENTION ctrl_surface = ANY HORIZONTAL SRFC DOWN IS POSITIVE , ANY VERTICAL SRFCE LEFT IS POSITIVE
	 // provide coefficients such that only dynamic pressure*area has to be multiplied to attain forces and moments in SI unit
	 // ALSO PROVIDE COFF. SUCH THAT SIGN CONVENTION IS FOLLOWED, SIGN CONVENTIONS ARE SUCH THAT...
	 // ON PROVIDING +VE ANGLE, WHAT DIRECTION MOMENT WILL GENERATE, IF MOMENT IS +VE , THEN COFF IS +VE, OTHERWISE VICE VERSA
     // v_define_ctrl_surface(type,L,D,Y,l,m,n,pwm_min, pwm_max, angle_min, angle_max,bandwidth(rad/sec),zeta,min_rate,max_rate);
	 v_define_ctrl_surface(AILERON_LEFT,0.0,0.0,0.0,-0.283f*b,0.0,0.052f*b,1100.0,1900.0,-20.0/57.3,20.0/57.3,40.0,0.7,-360.0/57.3f,360.0/57.3f);

	 v_define_ctrl_surface(AILERON_RIGHT,0,0,0,0.283f*b,0,-0.052f*b,1100,1900,-20/57.3,20/57.3,40.0,0.7,-360.0/57.3f,360.0/57.3f);

	 v_define_ctrl_surface(ELEVATOR_COMMON,0.38f,0.028f,0,0,-1.343f*c,0,1100,1900,-20/57.3,20/57.3,40.0,0.7,-360.0/57.3f,360.0/57.3f);

	 v_define_ctrl_surface(RUDDER_COMMON,0,0,0.1539f,0.005f*b,0,-0.044f*b,1100,1900,-20/57.3,20/57.3,40.0,0.7,-360.0/57.3f,360.0/57.3f);

	 csta[0] = AILERON_LEFT;
	 csta[1] = AILERON_RIGHT;
	 csta[2] = ELEVATOR_COMMON;
	 csta[3] = RUDDER_COMMON;
	 //csta[4] = ;


	 /*elevator_num = 2;
      aileron_num  = 2;
      rudder_num   = 2;

      extra_ctrl_srfc=0;
      extra_ctrl_srfc_aero_coff[10] = {0.0};*/

#endif

#ifdef Quad_H
	/*1cw       2ccw




	  4ccw      3cw
	*/
	int i=1;

	float x =1.0, y=1.0;

     x = 0.47f;
     y=  0.31f;

	 rotor_xyz[1-i][0]= x;                        rotor_xyz[2-i][0]= x;
	 rotor_xyz[1-i][1]=-y;                        rotor_xyz[2-i][1]= y;
	 rotor_xyz[1-i][2]=0.0;                       rotor_xyz[2-i][2]=0.0;

	 rotor_xyz[4-i][0]=-x;                        rotor_xyz[3-i][0]=-x;
	 rotor_xyz[4-i][1]=-y;                        rotor_xyz[3-i][1]= y;
	 rotor_xyz[4-i][2]=0.0;                       rotor_xyz[3-i][2]=0.0;


	 rotor_tilt[1-i][0]=0.0;                      rotor_tilt[2-i][0]=0.0;
	 rotor_tilt[1-i][1]=0.0;                      rotor_tilt[2-i][1]=0.0;
	 rotor_tilt[1-i][2]=0.0;                      rotor_tilt[2-i][2]=0.0;

	 rotor_tilt[4-i][0]=0.0;                      rotor_tilt[3-i][0]=0.0;
	 rotor_tilt[4-i][1]=0.0;                      rotor_tilt[3-i][1]=0.0;
	 rotor_tilt[4-i][2]=-180.0f/57.2958f;         rotor_tilt[3-i][2]=180.0f/57.2958f;

	 rotor_r_direction[1-i]=-1.0;                 rotor_r_direction[2-i]= 1.0;
	 rotor_r_direction[4-i]= 1.0;                 rotor_r_direction[3-i]=-1.0;


	 csta[0] = AILERON_LEFT;
	 csta[1] = AILERON_RIGHT;
	 csta[2] = ELEVATOR_COMMON;
	 csta[3] = RUDDER_COMMON;

#endif




#ifdef Quad_X

	 /*1cw       2ccw




	   4ccw      3cw
	 */

	int i=1; float x=1.0;
	x=0.47;

	 rotor_xyz[1-i][0]= x;                         rotor_xyz[2-i][0]=x;
	 rotor_xyz[1-i][1]=-x;                         rotor_xyz[2-i][1]=x;
	 rotor_xyz[1-i][2]=0.0;                        rotor_xyz[2-i][2]=0.0;

	 rotor_xyz[4-i][0]=-x;                         rotor_xyz[3-i][0]=-x;
	 rotor_xyz[4-i][1]=-x;                         rotor_xyz[3-i][1]= x;
	 rotor_xyz[4-i][2]=0.0;                        rotor_xyz[3-i][2]=0.0;

	 rotor_tilt[1-i][0]=0.0;                     rotor_tilt[2-i][0]=0.0;
	 rotor_tilt[1-i][1]=0.0;                     rotor_tilt[2-i][1]=0.0;
	 rotor_tilt[1-i][2]=-45.0f/57.2958f;         rotor_tilt[2-i][2]=45.0f/57.2958f;

	 rotor_tilt[4-i][0]=0.0;                     rotor_tilt[3-i][0]=0.0;
	 rotor_tilt[4-i][1]=0.0;                     rotor_tilt[3-i][1]=0.0;
	 rotor_tilt[4-i][2]=-135.0f/57.2958f;        rotor_tilt[3-i][2]=135.0f/57.2958f;

	 rotor_r_direction[1-i]=-1.0;                rotor_r_direction[2-i]= 1.0;
	 rotor_r_direction[4-i]= 1.0;                rotor_r_direction[3-i]=-1.0;
#endif






#ifdef Quad_+
	 /*
	        2ccw

	  1cw          3cw

	        4ccw
	  */
	 int i=1;
	 float x=1.0,y=1.0;

	 x=0.47f; y =0.47f;

	 rotor_xyz[1-i][0]=0.0;                       rotor_xyz[2-i][0]=x;
	 rotor_xyz[1-i][1]=-y;                        rotor_xyz[2-i][1]=0.0;
	 rotor_xyz[1-i][2]=0.0;                       rotor_xyz[2-i][2]=0.0;

	 rotor_xyz[4-i][0]=-x;                        rotor_xyz[3-i][0]=0.0;
	 rotor_xyz[4-i][1]=0.0;                       rotor_xyz[3-i][1]=y;
	 rotor_xyz[4-i][2]= 0.0;                      rotor_xyz[3-i][2]=0.0;

	 rotor_tilt[1-i][0]=0.0;                     rotor_tilt[2-i][0]=0.0;
	 rotor_tilt[1-i][1]=0.0;                     rotor_tilt[2-i][1]=0.0;
	 rotor_tilt[1-i][2]=-90.0f/57.2958f;         rotor_tilt[2-i][2]=0.0;

	 rotor_tilt[4-i][0]=0.0;                     rotor_tilt[3-i][0]=0.0;
	 rotor_tilt[4-i][1]=0.0;                     rotor_tilt[3-i][1]=0.0;
	 rotor_tilt[4-i][2]=180.0f/57.2958f;         rotor_tilt[3-i][2]=90.0f/57.2958f; ;

	 rotor_r_direction[1-i]=-1.0;                rotor_r_direction[2-i]= 1.0;
	 rotor_r_direction[4-i]= 1.0;                rotor_r_direction[3-i]=-1.0;
#endif




#ifdef Octa

	 uint8_t i=1;
     float x1,y1,x2,y2,z =0;

	    x2=1.37359f; y1 =1.37359f; z= 0.0;
	    x1 = 0.56895f;
	    y2 = 0.56895f;
	 	////////////////////////////////////////////////////////////////////////////////////////
		rotor_xyz[2-i][0]=  x2;                         rotor_xyz[3-i][0]=x2;
		rotor_xyz[2-i][1]= -y2;                         rotor_xyz[3-i][1]=y2;
		rotor_xyz[2-i][2]=-z;                         rotor_xyz[3-i][2]=-z;


		rotor_xyz[1-i][0]=   x1;                         rotor_xyz[4-i][0]=x1;
		rotor_xyz[1-i][1]=  -y1;                         rotor_xyz[4-i][1]=y1;
		rotor_xyz[1-i][2]=  z;                           rotor_xyz[4-i][2]= z  ;




		rotor_xyz[8-i][0]=-x1;                          rotor_xyz[5-i][0]=-x1;
		rotor_xyz[8-i][1]=-y1;                          rotor_xyz[5-i][1]= y1 ;
		rotor_xyz[8-i][2]=-z;                           rotor_xyz[5-i][2]= -z;


		rotor_xyz[7-i][0]=-x2;                          rotor_xyz[6-i][0]=-x2;
		rotor_xyz[7-i][1]=-y2;                           rotor_xyz[6-i][1]= y2;
		rotor_xyz[7-i][2]= z  ;                        rotor_xyz[6-i][2]= z  ;


	 	////////////////////////////////////////////////////////////////////////////////////////



	 	////////////////////////////////////////////////////////////////////////////////////////
	 	rotor_tilt[2-i][0]= 0.0;                     rotor_tilt[3-i][0]=0.0;
	 	rotor_tilt[2-i][1]= 0.0;                     rotor_tilt[3-i][1]=0.0;
	 	rotor_tilt[2-i][2]= 0.0;                     rotor_tilt[3-i][2]=0.0;

	 	rotor_tilt[1-i][0]= 0.0;                     rotor_tilt[4-i][0]=0.0;
	 	rotor_tilt[1-i][1]= 0.0;                     rotor_tilt[4-i][1]=0.0;
	 	rotor_tilt[1-i][2]= 0.0;                     rotor_tilt[4-i][2]=0.0;




	 	rotor_tilt[8-i][0]=0.0;                     rotor_tilt[5-i][0]=0.0;
	 	rotor_tilt[8-i][1]=0.0;                     rotor_tilt[5-i][1]=0.0;
	 	rotor_tilt[8-i][2]=0.0;                     rotor_tilt[5-i][2]=0.0;

	 	rotor_tilt[7-i][0]=0.0;                     rotor_tilt[6-i][0]=0.0;
	 	rotor_tilt[7-i][1]=0.0;                     rotor_tilt[6-i][1]=0.0;
	 	rotor_tilt[7-i][2]=0.0;                     rotor_tilt[6-i][2]=0.0;


	 	////////////////////////////////////////////////////////////////////////////////////////



	 	rotor_r_direction[2-i]= 1.0;                rotor_r_direction[3-i]=-1.0;
	 	rotor_r_direction[1-i]=-1.0;                rotor_r_direction[4-i]= 1.0;

	 	rotor_r_direction[8-i]= 1.0;                rotor_r_direction[5-i]=-1.0;
	 	rotor_r_direction[7-i]=-1.0;                rotor_r_direction[6-i]=1.0;

#endif



#ifdef Coax_Quad_X
	 // 1 2 3 4 are upward motors   5, 6, 7, 8 are downward motors
	 /*1cw        2ccw    - up motors
	   5ccw       6cw     - down motors

	   4ccw       3cw     - up motors
	   8cw        7ccw    - down motors
	 */
	int i=1;
	 float x=1.0,y=1.0;

	 x=0.47f; y =0.47f;
////////////////////////////////////////////////////////////////////////////////////////
	 rotor_xyz[1-i][0]= x;                       rotor_xyz[2-i][0]=x;
	 rotor_xyz[1-i][1]=-y;                       rotor_xyz[2-i][1]=y;
	 rotor_xyz[1-i][2]=0.0;                      rotor_xyz[2-i][2]=0.0;

	 rotor_xyz[5-i][0]= x;                       rotor_xyz[6-i][0]=x;
	 rotor_xyz[5-i][1]=-y;                       rotor_xyz[6-i][1]=y;
	 rotor_xyz[5-i][2]=0.0;                      rotor_xyz[6-i][2]=0.0;




	 rotor_xyz[4-i][0]=-x;                       rotor_xyz[3-i][0]=-x;
	 rotor_xyz[4-i][1]=-y;                       rotor_xyz[3-i][1]= y;
	 rotor_xyz[4-i][2]=0.0;                      rotor_xyz[3-i][2]=0.0;

	 rotor_xyz[8-i][0]=-x;                       rotor_xyz[7-i][0]=-x;
	 rotor_xyz[8-i][1]=-y;                       rotor_xyz[7-i][1]= y;
	 rotor_xyz[8-i][2]=0.0;                      rotor_xyz[7-i][2]=0.0;
////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////
	 rotor_tilt[1-i][0]=0.0;                     rotor_tilt[2-i][0]=0.0;
	 rotor_tilt[1-i][1]=0.0;                     rotor_tilt[2-i][1]=0.0;
	 rotor_tilt[1-i][2]=-45.0f/57.2958f;         rotor_tilt[2-i][2]=45.0f/57.2958f;

	 rotor_tilt[5-i][0]=0.0;                     rotor_tilt[6-i][0]=0.0;
	 rotor_tilt[5-i][1]=0.0;                     rotor_tilt[6-i][1]=0.0;
	 rotor_tilt[5-i][2]=-45.0f/57.2958f;         rotor_tilt[6-i][2]=45.0f/57.2958f;




	 rotor_tilt[4-i][0]=0.0;                     rotor_tilt[3-i][0]=0.0;
	 rotor_tilt[4-i][1]=0.0;                     rotor_tilt[3-i][1]=0.0;
	 rotor_tilt[4-i][2]=-135.0f/57.2958f;        rotor_tilt[3-i][2]=135.0f/57.2958f;

	 rotor_tilt[8-i][0]=0.0;                     rotor_tilt[7-i][0]=0.0;
	 rotor_tilt[8-i][1]=0.0;                     rotor_tilt[7-i][1]=0.0;
	 rotor_tilt[8-i][2]=-135.0f/57.2958f;        rotor_tilt[7-i][2]=135.0f/57.2958f;
////////////////////////////////////////////////////////////////////////////////////////



	 rotor_r_direction[1-i]=-1.0;                rotor_r_direction[2-i]= 1.0;
	 rotor_r_direction[5-i]= 1.0;                rotor_r_direction[6-i]=-1.0;

	 rotor_r_direction[4-i]= 1.0;                rotor_r_direction[3-i]=-1.0;
	 rotor_r_direction[8-i]=-1.0;                rotor_r_direction[7-i]= 1.0;
#endif







#ifdef Coax_Hexa_H_sym
	 /*
	        2ccw         3cw     - up motors
	         8cw         9ccw    - down motors
	          ##         ##
	          ##         ##
	          ##         ##
	        1cw  ####### 4ccw    - up motors
	        7ccw ####### 10cw    - down motors
	          ##         ##
	          ##         ##
	          ##         ##
	        6ccw         5cw     - up motors
	        12cw         11ccw   - down motors
	 */


	int i=1;
	float x=1.0,y=1.0, z=1.0;
	float offset = 0.0;
    float roll_tilt = 10.0/57.3;
	x=1.115f; y =0.953f; z= 0.145; offset = 0.01;
	////////////////////////////////////////////////////////////////////////////////////////
	rotor_xyz[2-i][0]= x;                         rotor_xyz[3-i][0]=x;
	rotor_xyz[2-i][1]=-y;                         rotor_xyz[3-i][1]=y;
	rotor_xyz[2-i][2]=-z;                         rotor_xyz[3-i][2]=-z;

	rotor_xyz[8-i][0]= x;                         rotor_xyz[9-i][0]=x;
	rotor_xyz[8-i][1]=-y;                         rotor_xyz[9-i][1]=y;
	rotor_xyz[8-i][2]= z + offset;                rotor_xyz[9-i][2]= z + offset;




	rotor_xyz[1-i][0]=0.0;                          rotor_xyz[4-i][0]=0.0;
	rotor_xyz[1-i][1]=-y;                           rotor_xyz[4-i][1]= y ;
	rotor_xyz[1-i][2]=-z;                           rotor_xyz[4-i][2]= -z;

	rotor_xyz[7-i][0]=0.0;                          rotor_xyz[10-i][0]=0.0;
	rotor_xyz[7-i][1]=-y;                           rotor_xyz[10-i][1]= y;
	rotor_xyz[7-i][2]= z + offset;                  rotor_xyz[10-i][2]= z + offset;




	rotor_xyz[6-i][0]=-x;                          rotor_xyz[5-i][0]=-x;
	rotor_xyz[6-i][1]=-y;                          rotor_xyz[5-i][1]= y;
	rotor_xyz[6-i][2]= -z;                         rotor_xyz[5-i][2]=  -z;

	rotor_xyz[12-i][0]=-x;                         rotor_xyz[11-i][0]=-x;
	rotor_xyz[12-i][1]=-y;                         rotor_xyz[11-i][1]= y;
	rotor_xyz[12-i][2]= z + offset;                rotor_xyz[11-i][2]= z + offset;
	////////////////////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////////////////////
	rotor_tilt[2-i][0]= 0.0;                     rotor_tilt[3-i][0]=0.0;
	rotor_tilt[2-i][1]= roll_tilt;               rotor_tilt[3-i][1]=-roll_tilt;
	rotor_tilt[2-i][2]= 0.0;                     rotor_tilt[3-i][2]=0.0;

	rotor_tilt[8-i][0]= 0.0;                     rotor_tilt[9-i][0]=0.0;
	rotor_tilt[8-i][1]= roll_tilt;               rotor_tilt[9-i][1]=-roll_tilt;
	rotor_tilt[8-i][2]= 0.0;                     rotor_tilt[9-i][2]=0.0;




	rotor_tilt[1-i][0]=0.0;                     rotor_tilt[4-i][0]=0.0;
	rotor_tilt[1-i][1]=-roll_tilt;              rotor_tilt[4-i][1]=roll_tilt;
	rotor_tilt[1-i][2]=0.0;                     rotor_tilt[4-i][2]=0.0;

	rotor_tilt[7-i][0]=0.0;                     rotor_tilt[10-i][0]=0.0;
	rotor_tilt[7-i][1]=-roll_tilt;              rotor_tilt[10-i][1]=roll_tilt;
	rotor_tilt[7-i][2]=0.0;                     rotor_tilt[10-i][2]=0.0;




	rotor_tilt[6-i][0]=0.0;                      rotor_tilt[5-i][0]=0.0;
	rotor_tilt[6-i][1]=roll_tilt;                rotor_tilt[5-i][1]=roll_tilt;
	rotor_tilt[6-i][2]=-180.0f/57.2958f;         rotor_tilt[5-i][2]=180.0f/57.2958f;

	rotor_tilt[12-i][0]=0.0;                     rotor_tilt[11-i][0]=0.0;
	rotor_tilt[12-i][1]=roll_tilt;               rotor_tilt[11-i][1]=roll_tilt;
	rotor_tilt[12-i][2]=-180.0f/57.2958f;        rotor_tilt[11-i][2]=180.0f/57.2958f;
	////////////////////////////////////////////////////////////////////////////////////////



	 rotor_r_direction[2-i]= 1.0;                rotor_r_direction[3-i]=-1.0;
	 rotor_r_direction[8-i]=-1.0;                rotor_r_direction[9-i]= 1.0;
	 
	 rotor_r_direction[1-i]=-1.0;                rotor_r_direction[4-i]= 1.0;
	 rotor_r_direction[7-i]= 1.0;                rotor_r_direction[10-i]=-1.0;

	 rotor_r_direction[6-i]= 1.0;                rotor_r_direction[5-i]=-1.0;
	 rotor_r_direction[12-i]=-1.0;               rotor_r_direction[11-i]= 1.0;
#endif
}




void v_define_ctrl_surface(AKAP_CONTROL_SURFACE_TYPE ctrl_srfc ,float lift,float drag,float sidef,float roll_m,float pitch_m,float yaw_m,float pwm_min, float pwm_max, float angle_min, float angle_max,float omega, float zeta, float min_rate, float max_rate)
{
	/*used in forces and moments*/
	actuator[ctrl_srfc].type = ctrl_srfc;
	actuator[ctrl_srfc].cL = lift;
	actuator[ctrl_srfc].cD = drag;
	actuator[ctrl_srfc].cY = sidef;
	actuator[ctrl_srfc].cl = roll_m;
	actuator[ctrl_srfc].cm = pitch_m;
	actuator[ctrl_srfc].cn = yaw_m;

	/*used in pwm to angle mapping*/
	actuator[ctrl_srfc].pwm_min = pwm_min;
	actuator[ctrl_srfc].pwm_max = pwm_max;
	actuator[ctrl_srfc].angle_min = angle_min;
	actuator[ctrl_srfc].angle_max = angle_max;

	/*used in setting limits*/
	actuator[ctrl_srfc].min_rate = min_rate;
	actuator[ctrl_srfc].max_rate = max_rate;

	actuator[ctrl_srfc].omega = omega;
	actuator[ctrl_srfc].zeta = zeta;
}
