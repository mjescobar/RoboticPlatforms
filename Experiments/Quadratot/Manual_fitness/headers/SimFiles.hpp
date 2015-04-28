#ifndef SIMFILES_HPP
#define SIMFILES_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <vector>
#include <boost/filesystem.hpp>
#include <ROBOTLIB>
#include "CalcFunctions.hpp"

using namespace ANN_USM;
using namespace std;
using namespace boost::filesystem;

class SimFiles
{
	ofstream file_joints_position;	
	ofstream file_joints_position_filtered;	
	ofstream file_joints_position_real;
	ofstream file_robot_position;
	ofstream file_joints_force;

	ofstream file_fitness;

	char * folder_path;

public:

	SimFiles(char * folder_path);
	~SimFiles();

	void openFitnessFile();
	void closeFitnessFile();
	void addFitnessToFile(int generation, int population, int fitness);

	void openNewJointsPositionFile(int generation, int population);
	void closeJointsPositionFile();
	void addFileJointsPosition(vector < Joint * > joints, double simulation_time);

	void openNewJointsForceFile(int generation, int population);
	void closeJointsForceFile();
	void addFileJointsForce(vector < Joint * > joints, double simulation_time);

	void openNewRobotPositionFile(int generation, int population);
	void closeRobotPositionFile();
	void addFileRobotPosition(Dummy * dummy, double simulation_time);
};

#endif