#ifndef FITNESS_HPP
#define FITNESS_HPP

#include <vector>
#include <cmath>
#include <iostream>
#include <ROBOTLIB>
#include "CalcFunctions.hpp"
#include "Simulation.hpp"

using namespace ANN_USM;
using namespace std;

#define PERFECT_FRECUENCY WAVE_FRECUENCY*2.0

class Fitness
{
	int jdcn;
	vector < int > jdcns;

	vector < double * > robot_position;

public:

	Fitness();
	~Fitness();

	void measuringValues(vector < Joint * > joints, Dummy * dummy);
	void resetValues();
	vector < double > getFrecuency();
	double getDistance();
	double getFrecuencyThreshold();

};

#endif