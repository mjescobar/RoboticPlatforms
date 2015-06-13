#ifndef FITNESS_HPP
#define FITNESS_HPP

#include <vector>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <ROBOTLIB>
#include "Simulation.hpp"
#include "CalcFunctions.hpp"

using namespace ANN_USM;
using namespace std;

#define FAILED_FITNESS 0.00001
#define FITNESS_BASE 0.0001

#define PERFECT_FRECUENCY WAVE_FRECUENCY*2.0
#define FRECUENCY_TOLERANCE 0.25

#define DISTANCE_OBJETIVE 0.0
#define ANGULAR_VELOCITY_OBJETIVE 0.15

//frequency fitness_1
//#define FREQUENCY_FITNESS(X) (double)exp(PERFECT_FRECUENCY -X)

//frequency fitness_2
#define AMPLITUDE (double)(10.0)
#define FREQUENCY_STDD 0.5
#define DISTANCE_STDD 0.08
#define ANGULAR_STDD 0.1
#define VAR_STDD 0.1

#define FREQUENCY_FITNESS(X) (double)(exp(-pow((X - PERFECT_FRECUENCY)/FREQUENCY_STDD, 2.0)/2.0)*AMPLITUDE + 0.000001)

#define DISTANCE_FITNESS(X) (double)(exp(-pow((X - DISTANCE_OBJETIVE)/DISTANCE_STDD, 2.0)/2.0)*AMPLITUDE + 0.000001)

#define VAR_FITNESS(X) (double)(exp(-pow((X)/VAR_STDD, 2.0)/2.0)*AMPLITUDE + 0.000001)

#define ANGULAR_VELOCITY_FITNESS_F(X) (double)(exp(-pow((X - ANGULAR_VELOCITY_OBJETIVE)/ANGULAR_STDD, 2.0)/2.0)*AMPLITUDE + 0.000001)
#define ANGULAR_VELOCITY_FITNESS(X) (double)((X < ANGULAR_VELOCITY_OBJETIVE) ? ANGULAR_VELOCITY_FITNESS_F(X) : AMPLITUDE + 0.000001)

class Fitness
{
	int jdcn;
	vector < int > jdcns;
	vector < double > fitness;

	double frecuency;
	double final_fitness;
	double distance;

	vector < double * > robot_position;
	vector < double > robot_vx;
	vector < double > robot_vy;
	vector < double > robot_walpha;
	vector < double > generation_frecuency;
	vector < double > generation_fitness;
	
	// int iteration;

public:

	Fitness();
	~Fitness();

	void measuringValues(vector < Joint * > joints, Dummy * dummy);
	double calcFitness();
	void resetPopulationValues();
	void resetGenerationValues();
	double getFrecuency();
	double getFitness();
	double getDistance();
	vector < double > getGenerationFrecuency();
	vector < double > getGenerationFitness();
	double getFrecuencyThreshold();
	double getJointDirectionChangeNumber();
	string getFitnessResults();

};

#endif