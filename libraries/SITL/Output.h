#ifndef OUTPUT_H_
#define OUTPUT_H_

#include <stdio.h>

// Initialize the log file (opens/creates the file and writes the CSV header)
void v_output_log_init();

// Write one row of simulation data to the log file
void v_output_log_write(float t);

// Close the log file cleanly
void v_output_log_close();

#endif // OUTPUT_H_
