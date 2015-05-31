#ifndef FITNESS_CPP
#define FITNESS_CPP

#include "Fitness.hpp"

Fitness::Fitness()
{
	jdcn = 0;
	jdcns = vector < int > ((int)(N_LEGS + GRA_LIB_EXT),0);
	fitness = vector < double > (6,FAILED_FITNESS);

	frecuency = 0;
	final_fitness = FAILED_FITNESS;
	distance = 0;
}

Fitness::~Fitness()
{

}

void Fitness::measuringValues(vector < Joint * > joints, Dummy * dummy)
{
	double * position = new double[3];
	double * lVelocity = new double[3];
	double * aVelocity = new double[3];

	dummy->getPosition(-1, position);
	dummy->getVelocity(lVelocity, aVelocity);

	robot_position.push_back(position);
	robot_vx.push_back(lVelocity[0]);
	robot_vy.push_back(lVelocity[1]);
	robot_walpha.push_back(aVelocity[0]);

	/*
	for(int i = 0; i < (int)joints.size(); i++)
		if(joints.at(i)->getJointChangeDirection())
			jdcn++;
	*/

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

double Fitness::calcFitness()
{	
	if(jdcn != 0)
	{
		double * final_location = robot_position.back();
		double * initial_location = robot_position.front();
				
		vector < double > freq ((int)(N_LEGS + GRA_LIB_EXT),0.0);

		for(int i = 0; i < N_LEGS; i++)
			freq.at(i) = FREQUENCY_FITNESS((double)(jdcns.at(i)/GRA_LIB)/(TIME_SIMULATION - TIME_INIT_MEASURING));

		freq.back() = FREQUENCY_FITNESS((double)jdcns.back()/(TIME_SIMULATION - TIME_INIT_MEASURING));

		frecuency = mean(freq);
		distance = sqrt(pow(final_location[0]- initial_location[0],2) + pow(final_location[1]- initial_location[1],2));

		fitness.at(0) = DISTANCE_FITNESS(distance);
		fitness.at(1) = frecuency;
		fitness.at(2) = VAR_FITNESS(var(robot_vx));
		fitness.at(3) = VAR_FITNESS(var(robot_vy));
		fitness.at(4) = VAR_FITNESS(var(robot_walpha));
		fitness.at(5) = ANGULAR_VELOCITY_FITNESS(mean(robot_walpha));
		
		final_fitness = min(fitness);

		generation_frecuency.push_back(frecuency);
		generation_fitness.push_back(final_fitness);		
	}
	else final_fitness = FAILED_FITNESS;

	return final_fitness;		
}

void Fitness::resetPopulationValues()
{
	jdcn = 0;
	jdcns = vector < int > ((int)(N_LEGS + GRA_LIB_EXT),0);
	fitness = vector < double > (6,FAILED_FITNESS);

	frecuency = 0;
	final_fitness = FAILED_FITNESS;
	distance = 0;
	robot_position.clear();
	robot_vx.clear();
	robot_vy.clear();
	robot_walpha.clear();
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
	return final_fitness;
}

double Fitness::getDistance()
{
	return distance;
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
	return PERFECT_FRECUENCY;
}

double Fitness::getJointDirectionChangeNumber()
{
	return jdcn;
}

string Fitness::getFitnessResults()
{
	stringstream results;

	results << endl;

	results << "Fitness distance  \t" << fitness.at(0) << endl;
	results << "Fitness frecuency \t" << fitness.at(1) << endl;
	results << "Fitness var robot_vx  \t" << fitness.at(2) << endl;
	results << "Fitness var robot_vy  \t" << fitness.at(3) << endl;
	results << "Fitness var robot_walpha \t" << fitness.at(4) << endl;
	results << "Fitness magnitude walpha \t" << fitness.at(5) << endl;
	results << "Fitness final:\t" << final_fitness << endl << endl;

	return results.str();
}

#endif