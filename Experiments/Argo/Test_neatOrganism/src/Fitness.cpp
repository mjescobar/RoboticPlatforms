#ifndef FITNESS_CPP
#define FITNESS_CPP

#include "Fitness.hpp"

Fitness::Fitness()
{
	jdcn = 0;
	frecuency = 0;
	fitness = FAILED_FITNESS;
	distance = 0;
	penalized_distance = 0;
}

Fitness::~Fitness()
{

}

void Fitness::measuringValues(vector < Joint * > joints, Dummy * dummy)
{
	double position[3];

	dummy->getPosition(-1, position);

	double * aux = new double[3];

	for(int i = 0; i < 3 ; i++)
		aux[i] = position[i];

	robot_position.push_back(aux);

	for(int i = 0; i < (int)joints.size(); i++)
		if(joints.at(i)->getJointChangeDirection())
			jdcn++;
}

double Fitness::calcFitness()
{	
	if(jdcn != 0)
	{
		double * final_location = robot_position.back();
		double * initial_location = robot_position.front();
				
		frecuency = (double)(jdcn/(N_LEGS*GRA_LIB + GRA_LIB_EXT))/(TIME_SIMULATION - TIME_INIT_MEASURING);		
		distance = sqrt(pow(final_location[0]- initial_location[0],2) + pow(final_location[1]- initial_location[1],2));
		penalized_distance = distance*F_DISTANCE_PENALIZATION(frecuency);
		fitness = FITNESS_FUNCTION(penalized_distance);
		generation_frecuency.push_back(frecuency);
		generation_fitness.push_back(fitness);

	}
	else fitness = FAILED_FITNESS;

	return fitness;		
}

void Fitness::resetPopulationValues()
{
	jdcn = 0;
	frecuency = 0;
	fitness = FAILED_FITNESS;
	distance = 0;
	penalized_distance = 0;
	robot_position.clear();
}

void Fitness::resetGenerationValues()
{
	generation_fitness.clear();
	generation_frecuency.clear();
}

double Fitness::getFrecuency()
{
	return frecuency;
}

double Fitness::getFitness()
{
	return fitness;
}

double Fitness::getDistance()
{
	return distance;
}

double Fitness::getDistancePenalization()
{
	return F_DISTANCE_PENALIZATION(frecuency);
}

double Fitness::getPenalizedDistance()
{
	return penalized_distance;
}

vector < double > Fitness::getGenerationFrecuency()
{
	return generation_frecuency;
}

vector < double > Fitness::getGenerationFitness()
{
	return generation_fitness;
}

double Fitness::getFrecuencyThreshold()
{
	return PERFECT_FRECUENCY*(1 + FRECUENCY_TOLERANCE);
}

double Fitness::getJointDirectionChangeNumber()
{
	return jdcn;
}

#endif