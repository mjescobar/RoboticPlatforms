#ifndef FITNESS_HPP
#define FITNESS_HPP

#include <vector>
#include <cmath>
#include <iostream>
#include <ROBOTLIB>
#include "Simulation.hpp"
#include "CalcFunctions.hpp"

using namespace ANN_USM;
using namespace std;

#define FAILED_FITNESS 0.00001
#define FITNESS_BASE 0.0001

#define PERFECT_FRECUENCY WAVE_FRECUENCY*2.0
#define FRECUENCY_TOLERANCE 0.25
#define DISTANCE_OBJETIVE 1.1
//frequency fitness_1
//#define FREQUENCY_FITNESS(X) (double)exp(PERFECT_FRECUENCY -X)

//frequency fitness_2
#define AMPLITUDE (double)(10.0)
#define GAUSSIAN_STDD 0.5
#define FREQUENCY_FITNESS(X) (double)(exp(-pow((X - PERFECT_FRECUENCY)/GAUSSIAN_STDD, 2.0)/2.0)*AMPLITUDE/sqrt(2*M_PI*GAUSSIAN_STDD*GAUSSIAN_STDD) + 0.000001)


#define DISTANCE_FITNESS(X) (double)(exp(-pow((X - DISTANCE_OBJETIVE)/GAUSSIAN_STDD, 2.0)/2.0)*AMPLITUDE/sqrt(2*M_PI*GAUSSIAN_STDD*GAUSSIAN_STDD) + 0.000001)


class Fitness
{
	int jdcn;
	vector < int > jdcns;
	vector < double > fitness;

	double frecuency;
	double final_fitness;
	double distance;

	vector < double * > robot_position;
	vector < double > generation_frecuency;
	vector < double > generation_fitness;

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


};

#endif