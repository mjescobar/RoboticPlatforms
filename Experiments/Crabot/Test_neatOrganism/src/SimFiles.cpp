#ifndef SIMFILES_CPP
#define SIMFILES_CPP

#include "SimFiles.hpp"

SimFiles::SimFiles()
{
	file_results.open("simulation_files/results.txt");
	file_fitness.open("simulation_files/fitness.txt");
	file_frecuency.open("simulation_files/frecuency.txt");
}

SimFiles::~SimFiles()
{
	file_results.close();
	file_fitness.close();
	file_frecuency.close();
}
	
void SimFiles::addFileResults(Fitness * fitness, int generation, int population)	
{	
	file_results << endl << "===========================================   G" << generation << " P" << population <<endl;
	file_results << "Joint direction change number: " << fitness->getJointDirectionChangeNumber() << endl;
	file_results << "Joint distance change number frecuency: " << fitness->getFrecuency() << endl;
	file_results << "Traveled distance : " << fitness->getDistance() << endl;
	file_results << "Distance penalization : " << fitness->getDistancePenalization() << endl;
	file_results << "Penalized distance : " << fitness->getPenalizedDistance() << endl;
	file_results << "Fitness: " << fitness->getFitness() << endl;
}

void SimFiles::addFileFitness(Fitness * fitness, int generation)
{
	double average = mean(fitness->getGenerationFitness());
	double stddesv = stdDesviation(fitness->getGenerationFitness());

	file_fitness << generation << "\t" << average << "\t" << stddesv << endl;
}

void SimFiles::addFileFrecuency(Fitness * fitness, int generation)
{
	double average = mean(fitness->getGenerationFrecuency());
	double stddesv = stdDesviation(fitness->getGenerationFrecuency());

	file_frecuency << generation << "\t" << average << "\t" << stddesv << "\t" << fitness->getFrecuencyThreshold() << endl;
}

void SimFiles::openNewJointsPositionFile(int generation, int population)
{
	stringstream file_name;
	file_name << "simulation_files/joints_position/jointsPosition_G" << generation << "_P" << population << ".txt";
	file_joints_position.open((char*)file_name.str().c_str());
}

void SimFiles::closeJointsPositionFile()
{
	file_joints_position.close();
}

void SimFiles::addFileJointsPosition(vector < Joint * > joints, double simulation_time)
{
	file_joints_position << simulation_time;

	for(int i = 0; i < (int)joints.size(); i++)
		file_joints_position << "\t" << joints.at(i)->getJointGoalPosition();

	file_joints_position << endl;
}

void SimFiles::openNewRobotPositionFile(int generation, int population)
{
	stringstream file_name;
	file_name << "simulation_files/robot_position/robotPosition_G" << generation << "_P" << population << ".txt";
	file_robot_position.open((char*)file_name.str().c_str());
}

void SimFiles::closeRobotPositionFile()
{
	file_robot_position.close();	
}

void SimFiles::addFileRobotPosition(Dummy * dummy, double simulation_time)
{
	double position[3];
	dummy->getPosition(-1, position);

	file_robot_position << simulation_time;

	for(int i = 0; i < 3; i++)
		file_robot_position << "\t" << position[i];

	file_robot_position << endl;
}

#endif