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
	file_results << "======================================  G" << generation << " P" << population <<endl;
	file_results << fitness->getFitnessResults(generation) << endl;
}

void SimFiles::addFileResults(double best_fitness)
{
	file_results << endl << "BEST RESULT ------------------------------------" << endl << endl;
	file_results << "\t-> " << best_fitness << endl << endl;
}

void SimFiles::addFileFitness(Fitness * fitness, int generation)
{
	double average = mean(fitness->getGenerationFitness());
	double stddesv = stdDesviation(fitness->getGenerationFitness());

	file_fitness << generation << "\t" << average << "\t" << stddesv << endl;
}

void SimFiles::addFileFitness(vector < Fitness * > fitness, int generation)
{
	vector < double > fit;
	for(int i = 0; i < (int)fitness.size(); i++)
	{
		vector < double > aux(fitness.at(i)->getGenerationFitness());
		fit.insert(fit.end(),aux.begin(),aux.end());
	}

	double average = mean(fit);
	double stddesv = stdDesviation(fit);

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

	stringstream file_name2;
	file_name2 << "simulation_files/joints_position_real/jointsPosition_G" << generation << "_P" << population << ".txt";
	file_joints_position_real.open((char*)file_name2.str().c_str());

	stringstream file_name3;
	file_name3 << "simulation_files/joints_position_filtered/jointsPosition_G" << generation << "_P" << population << ".txt";
	file_joints_position_filtered.open((char*)file_name3.str().c_str());
}

void SimFiles::closeJointsPositionFile()
{
	file_joints_position.close();
	file_joints_position_real.close();
	file_joints_position_filtered.close();
}

void SimFiles::addFileJointsPosition(vector < Joint * > joints, double simulation_time)
{
	file_joints_position_real << simulation_time;

	for(int i = 0; i < (int)joints.size(); i++)
		file_joints_position_real << "\t" << joints.at(i)->getJointCurrentPosition();

	file_joints_position_real << endl;

	file_joints_position << simulation_time;

	for(int i = 0; i < (int)joints.size(); i++)
		file_joints_position << "\t" << joints.at(i)->getJointGoalPosition();

	file_joints_position << endl;

	file_joints_position_filtered << simulation_time;

	for(int i = 0; i < (int)joints.size(); i++)
		file_joints_position_filtered << "\t" << joints.at(i)->getJointFilteredPosition();

	file_joints_position_filtered << endl;
}

void SimFiles::openNewJointsForceFile(int generation, int population)
{
	stringstream file_name;
	file_name << "simulation_files/joints_force/jointsForce_G" << generation << "_P" << population << ".txt";
	file_joints_force.open((char*)file_name.str().c_str());
}

void SimFiles::closeJointsForceFile()
{
	file_joints_force.close();
}

void SimFiles::addFileJointsForce(vector < Joint * > joints, double simulation_time)
{
	file_joints_force << simulation_time;

	for(int i = 0; i < (int)joints.size(); i++)
		file_joints_force << "\t" << joints.at(i)->getJointForce();

	file_joints_force << endl;
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