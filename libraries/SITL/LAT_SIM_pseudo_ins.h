#include "stdio.h"


typedef struct
{
float u;
float v;
float w;
float phi;
float theta;
float psi;
float p;
float q;
float r;
float lat;
float longt;
float z;
float ax;
float ay;
float az;

}sensor_accuracy_struct;

extern sensor_accuracy_struct sensor_accuracy , low_freq_inacc_component, high_freq_noise_component , low_freq_inacc_component_old, high_freq_noise_component_old;
extern sensor_accuracy_struct sensor_inacc_lpff , sensor_hf_noise_lpff, sensor_noise_hf_amp;

extern int flag_sensor_input_inacc, flag_sensor_input_hfnoise;

extern void lat_long_to_xy(float latitude_point,float longitude_point,float Alt,float* pos);
extern void fn_low_frequency_sensor_inacc_modeler(float);
extern void fn_high_frequency_sensor_noise_modeler(float);
extern void fn_sensor_delay_input();
extern void fn_sensor_initialise_pseudo_ins();
