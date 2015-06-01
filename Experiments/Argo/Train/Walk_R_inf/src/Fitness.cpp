#ifndef FITNESS_CPP
#define FITNESS_CPP

#include "Fitness.hpp"

Fitness::Fitness()
{
	jdcn = 0;
	jdcns = vector < int > ((int)(N_LEGS + GRA_LIB_EXT),0);
	fitness = vector < double > (4,FAILED_FITNESS);

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
	fitness = vector < double > (4,FAILED_FITNESS);

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


	// results << "distance \t" << fitness.at(0) << endl;
	// results << "frecuency \t" << fitness.at(1) << endl;
	// results << "var vel x\t" << fitness.at(2) << endl;
	// results << "var vel y\t" << fitness.at(3) << endl;
	// results << "vaw W\t" << fitness.at(4) << endl;
	// results << "|W|\t" << fitness.at(5) << endl;
	// results << "mean W: " << mean(robot_walpha) << endl;
	// results << "Fitness final:\t" << final_fitness << endl << endl;

	results << "fitness distance \t" << fitness.at(0) << endl;
	results << "fitness frecuency \t" << fitness.at(1) << endl;
	results << "var Vx\t" << fitness.at(2) << endl;
	results << "var Vy\t" << fitness.at(3) << endl;
	results << "distance: " << distance << endl;
	results << "mean Vx: " << mean(robot_vx) << endl;
	results << "mean Vy: " << mean(robot_vy) << endl;
	results << "Fitness final:\t" << final_fitness << endl << endl;

	return results.str();
}

#endif