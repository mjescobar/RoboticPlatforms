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

//fitness_1
//#define F_DISTANCE_PENALIZATION(X) (double)exp(PERFECT_FRECUENCY -X)

//fitness_2
#define AMPLITUDE (double)(1.0 + FRECUENCY_TOLERANCE)
#define GAUSSIAN_STDD 1.0
#define F_DISTANCE_PENALIZATION(X) (double)(exp(-pow((X - PERFECT_FRECUENCY)/GAUSSIAN_STDD, 2.0)/2.0)*AMPLITUDE)

#define FITNESS_FUNCTION(X) (double)(pow(2.0,pow(X,2.0)) - 1 + FITNESS_BASE)

class Fitness
{
	int jdcn;
	vector < int > jdcns;

	double frecuency;
	double fitness;
	double distance;
	double distance_penalization;
	double penalized_distance;

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
	double getDistancePenalization();
	double getPenalizedDistance();
	vector < double > getGenerationFrecuency();
	vector < double > getGenerationFitness();
	double getFrecuencyThreshold();
	double getJointDirectionChangeNumber();


};

#endif