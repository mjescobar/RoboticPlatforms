#ifndef FITNESS_CPP
#define FITNESS_CPP

#include "Fitness.hpp"

Fitness::Fitness()
{
	jdcn = 0;
	jdcns = vector < int > ((int)(N_LEGS + GRA_LIB_EXT),0);
}

Fitness::~Fitness()
{

}

void Fitness::measuringValues(vector < Joint * > joints, Dummy * dummy)
{
	double * position = new double[3];

	dummy->getPosition(-1, position);
	
	robot_position.push_back(position);

	for(int i = 0; i < (int)joints.size()-1; i++)
		if(joints.at(i)->getJointChangeDirection())
		{
			jdcns.at(i%4)++;
			jdcn++;
		}

	if(joints.back()->getJointChangeDirection())
	{
		jdcns.back()++;
		jdcn++;
	}
}

void Fitness::resetValues()
{
	jdcn = 0;
	jdcns = vector < int > ((int)(N_LEGS + GRA_LIB_EXT),0);

	robot_position.clear();
}

vector < double > Fitness::getFrecuency()
{
	vector < double > freq ((int)(N_LEGS + GRA_LIB_EXT),0.0);

	for(int i = 0; i < N_LEGS; i++)
			freq.at(i) = (double)(jdcns.at(i)/GRA_LIB)/(TIME_SIMULATION - TIME_INIT_MEASURING);

	freq.back() = (double)jdcns.back()/(TIME_SIMULATION - TIME_INIT_MEASURING);

	return freq;
}

double Fitness::getDistance()
{
	double * final_location = robot_position.back();
	double * initial_location = robot_position.front();

	return sqrt(pow(final_location[0]- initial_location[0],2) + pow(final_location[1]- initial_location[1],2));
}

double Fitness::getFrecuencyThreshold()
{
	return PERFECT_FRECUENCY;
}

#endif