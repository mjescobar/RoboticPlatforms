#ifndef SIMFILES_CPP
#define SIMFILES_CPP

#include "SimFiles.hpp"

SimFiles::SimFiles(char * folder_path)
{
	int size = strlen(folder_path)+1;
	this->folder_path = new char[size];
	strncpy(this->folder_path, folder_path, size);
}

SimFiles::~SimFiles()
{
	
}
	
void SimFiles::openFitnessFile()
{
	stringstream filename;
	filename << folder_path << "/fitness.txt";
	file_fitness.open((char*)filename.str().c_str(), fstream::in | fstream::app);
}

void SimFiles::closeFitnessFile()
{
	file_fitness.close();
}

void SimFiles::addFitnessToFile(int generation, int population, int fitness)
{
	file_fitness << generation << "\t" << population << "\t" << fitness << endl;	
}

void SimFiles::openInputFitnessFile()
{
	stringstream filename;
	filename << folder_path << "/input_fitness.txt";
	file_input_fitness.open((char*)filename.str().c_str(), fstream::in | fstream::app);
}

void SimFiles::closeInputFitnessFile()
{
	file_input_fitness.close();
}

void SimFiles::addiInputFitnessToFile(int generation, int population, Fitness * fitness)
{
	vector < double > freq(fitness->getFrecuency());
	file_input_fitness << generation << "\t" << population << "\t" << fitness->getDistance() << "\t" << fitness->getFrecuencyThreshold();	
	for(int i = 0; i < (int)freq.size(); i++)
		file_input_fitness << "\t" << freq.at(i);
	file_input_fitness << endl;
}

void SimFiles::openNewJointsPositionFile(int generation, int population)
{
	stringstream path0, path1, path2, path3;
	path0 << folder_path << "/simulation_files" << flush;
	path dir0((char*)path0.str().c_str());
	create_directory(dir0);

	path1 << folder_path << "/simulation_files/joints_position/" << flush;
	path2 << folder_path << "/simulation_files/joints_position_real/" << flush;
	path3 << folder_path << "/simulation_files/joints_position_filtered/" << flush;

	path dir1((char*)path1.str().c_str());
	path dir2((char*)path2.str().c_str());
	path dir3((char*)path3.str().c_str());
	
	create_directory(dir1);
	create_directory(dir2);
	create_directory(dir3);

	

	stringstream file_name;
	file_name << folder_path << "/simulation_files/joints_position/jointsPosition_G" << generation << "_P" << population << ".txt";
	file_joints_position.open((char*)file_name.str().c_str());

	stringstream file_name2;
	file_name2 << folder_path << "/simulation_files/joints_position_real/jointsPosition_G" << generation << "_P" << population << ".txt";
	file_joints_position_real.open((char*)file_name2.str().c_str());

	stringstream file_name3;
	file_name3 << folder_path << "/simulation_files/joints_position_filtered/jointsPosition_G" << generation << "_P" << population << ".txt";
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