#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <cmath>

#define TIME_SIMULATION 6.0
#define TIME_INIT_MEASURING 1.0

#define N_LEGS 4.0
#define GRA_LIB 2
#define GRA_LIB_EXT 1
#define ADITIONAL_HYPERNEAT_INPUTS 2
#define RAD (double)(M_PI/180.0)

/*// FOR FIRST TEST
#define MAX_ANGLE_INNER 60.0*RAD
#define MIN_ANGLE_INNER -85.0*RAD
#define MAX_ANGLE_OUTER 39.0*RAD
#define MIN_ANGLE_OUTER -113.0*RAD
#define MAX_ANGLE_CENTER 23.0*RAD
#define MIN_ANGLE_CENTER -23.0*RAD
*/

//FOR SECOND TEST
#define MAX_ANGLE_INNER 45.0*RAD
#define MIN_ANGLE_INNER -90.0*RAD
#define MAX_ANGLE_OUTER 45.0*RAD
#define MIN_ANGLE_OUTER -90.0*RAD
#define MAX_ANGLE_CENTER 23.0*RAD
#define MIN_ANGLE_CENTER -23.0*RAD

#define MAX_ANGLE_LIMIT {MAX_ANGLE_INNER,	MAX_ANGLE_INNER,	MAX_ANGLE_INNER,	MAX_ANGLE_INNER,	MAX_ANGLE_OUTER,	MAX_ANGLE_OUTER,	MAX_ANGLE_OUTER,	MAX_ANGLE_OUTER,	MAX_ANGLE_CENTER}
#define MIN_ANGLE_LIMIT {MIN_ANGLE_INNER,	MIN_ANGLE_INNER,	MIN_ANGLE_INNER,	MIN_ANGLE_INNER,	MIN_ANGLE_OUTER,	MIN_ANGLE_OUTER,	MIN_ANGLE_OUTER,	MIN_ANGLE_OUTER,	MIN_ANGLE_CENTER}

#define INIT_ANGLE_INNER -30.0*RAD
#define INIT_ANGLE_OUTER -60.0*RAD
#define INIT_ANGLE_CENTER 0.0*RAD
#define INITIAL_ANGLES {INIT_ANGLE_INNER,	INIT_ANGLE_INNER,	INIT_ANGLE_INNER,	INIT_ANGLE_INNER,	INIT_ANGLE_OUTER,	INIT_ANGLE_OUTER,	INIT_ANGLE_OUTER,	INIT_ANGLE_OUTER,	INIT_ANGLE_CENTER}

#define FRECUENCY_CMD 160.0
#define STEP_CALC 4.0

#define DELTA_T (double)(1.0/FRECUENCY_CMD)

#define WAVE_FRECUENCY 1.32//0.64
#define SIN(X,PHI) (double)sin(2.0*M_PI*WAVE_FRECUENCY*X + PHI*M_PI/2.0)

//Caracteristicas_del_motor AX ------------------------------------------------------------
#define ANGULO_MAX (300.0*M_PI/180)
#define ANGULO_RESOLUCION 1023.0
#define VELOCIDAD_ANGULAR_MAX 11.938051
#define RESOLUCION_VELOCIDAD 1023.0
#define TRANSFORMAR_VELOCIDAD_A_NUMERO(X) (X*(RESOLUCION_VELOCIDAD/VELOCIDAD_ANGULAR_MAX))
#define TRANSFORMAR_NUMERO_A_VELOCIDAD(X) (X*(VELOCIDAD_ANGULAR_MAX/RESOLUCION_VELOCIDAD))
#define TRANSFORMAR_A_RADIANES(X) (X*(ANGULO_MAX/ANGULO_RESOLUCION) - ANGULOCERO)*M_PI/180.0
#define TRANSFORMAR_A_NUMEROS(X) ((X + ANGULOCERO*M_PI/180.0)*(ANGULO_RESOLUCION/ANGULO_MAX))
#define ANGULOCERO 150.0
#define VEL_MOT_DEF 900
//------------------------------------------------------------------------------------------

#endif