#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <cmath>

#define TIME_SIMULATION 10.0
#define TIME_INIT_MEASURING 2.0

#define N_LEGS 4.0
#define GRA_LIB 2
#define GRA_LIB_EXT 1
#define ADITIONAL_HYPERNEAT_INPUTS 2
#define RAD (double)M_PI/180.0

#define MAX_ANGLE_INNER 60.0*RAD
#define MIN_ANGLE_INNER -85.0*RAD
#define MAX_ANGLE_OUTER 39.0*RAD
#define MIN_ANGLE_OUTER -113.0*RAD
#define MAX_ANGLE_CENTER 23.0*RAD
#define MIN_ANGLE_CENTER -23.0*RAD

#define MAX_ANGLE_LIMIT {MAX_ANGLE_INNER,	MAX_ANGLE_INNER,	MAX_ANGLE_INNER,	MAX_ANGLE_INNER,	MAX_ANGLE_OUTER,	MAX_ANGLE_OUTER,	MAX_ANGLE_OUTER,	MAX_ANGLE_OUTER,	MAX_ANGLE_CENTER}
#define MIN_ANGLE_LIMIT {MIN_ANGLE_INNER,	MIN_ANGLE_INNER,	MIN_ANGLE_INNER,	MIN_ANGLE_INNER,	MIN_ANGLE_OUTER,	MIN_ANGLE_OUTER,	MIN_ANGLE_OUTER,	MIN_ANGLE_OUTER,	MIN_ANGLE_CENTER}

#define FRECUENCY_CMD 160.0
#define STEP_CALC 4.0

#define DELTA_T (double)(1.0/FRECUENCY_CMD)

#endif