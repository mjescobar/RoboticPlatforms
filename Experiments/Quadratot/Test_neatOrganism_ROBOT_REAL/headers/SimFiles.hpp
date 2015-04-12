#ifndef SIMFILES_HPP
#define SIMFILES_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <vector>
#include <ROBOTLIB>
#include "Fitness.hpp"
#include "CalcFunctions.hpp"

using namespace ANN_USM;
using namespace std;

class SimFiles
{
	ofstream file_results;
	ofstream file_fitness;
	ofstream file_frecuency;
	ofstream file_joints_position;
	ofstream file_robot_position;

public:

	SimFiles();
	~SimFiles();

	void addFileResults(Fitness * fitness, int generation, int population);
	void addFileFitness(Fitness * fitness, int generation);
	void addFileFrecuency(Fitness * fitness, int generation);

	void openNewJointsPositionFile(int generation, int population);
	void closeJointsPositionFile();
	void addFileJointsPosition(vector < Joint * > joints, double simulation_time);

	void openNewRobotPositionFile(int generation, int population);
	void closeRobotPositionFile();
	void addFileRobotPosition(Dummy * dummy, double simulation_time);
};

#endif