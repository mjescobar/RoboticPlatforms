#ifndef Quadratot_CPP
#define Quadratot_CPP

#include "Quadratot.hpp"

int main(int argc, char* argv[])
{	
	srand (time(0));	

	CM700 * cm700 = new CM700(GRA_LIB*N_LEGS+GRA_LIB_EXT,GRA_LIB);
	SimFiles * simfile = new SimFiles(); 

	if(argc < 4)
	{
		cerr << "ERROR: The number of arguments is incorrect" << endl;
		return -1;	
	} 
	
	// ============= VREP INITIALIZATIONS ============= //			

	vector < Joint * > joints;

	double max_angle_limit[] = MAX_ANGLE_LIMIT;
	double min_angle_limit[] = MIN_ANGLE_LIMIT;
	double init_angle[] = INITIAL_ANGLES;

	for(int i = 0; i < N_LEGS*GRA_LIB + GRA_LIB_EXT; i++)
		joints.push_back(new Joint(cm700, (char*)"AX", i+1, max_angle_limit[i], min_angle_limit[i], (char*)"SCALE", init_angle[i]));
	
	// ================================================ //

	// ========== HYPERNEAT INITIALIZATIONS =========== //

	vector < double * > next;
	vector < double * > pass;

	for(int i = 0; i < N_LEGS*GRA_LIB + GRA_LIB_EXT; i++)
	{
		double joint_pos = joints.at(i)->getJointInitialPosition();

		double * next_pos = new double(joint_pos);
		double * pass_pos = new double(joint_pos);

		next.push_back(next_pos);
		pass.push_back(pass_pos);
	}

	for(int i = 0; i < ADITIONAL_HYPERNEAT_INPUTS; i++)
	{
		double * aux_pass = new double(SIN(0,i));
		pass.push_back(aux_pass);
	}

	HyperNeat * hyperneat = new HyperNeat(pass, next, argv[1], argv[2], argv[3]);

	// ================================================ //

	double sim_time = 0;					
	int step = 0;
	vector < double > sum_next ((int)joints.size(),0.0);

	simfile->openNewJointsPositionFile(0, 0);

	if(!hyperneat->CreateSubstrateConnections(argv[3])) 
	{
		clog << "ERROR: Neat organism has not created substrate connections successfully" << endl;
		return(0);
	} 

	for(int i = 0; i < (int)joints.size(); i++)
	{
		double joint_pos = joints.at(i)->getJointInitialPosition();
		joints.at(i)->setJointInitialPosition();
		*next.at(i) = joint_pos;
		*pass.at(i) = joint_pos;
	}

	cm700->moveAll();

	cout << "Press Enter key to init program..." << endl;
	cin.get();

	while(sim_time < TIME_SIMULATION)
	{						
		for(int i = 0; i < ADITIONAL_HYPERNEAT_INPUTS; i++)
		{
			*pass.at((int)joints.size()+i) = SIN(sim_time,i);
		}

		hyperneat->EvaluateSubstrateConnections();

		for(int i = 0; i < (int)joints.size(); i++)
		{
			sum_next.at(i) = sum_next.at(i) + *next.at(i);
			*pass.at(i) = *next.at(i);
		}		
		step++;
		
		if(step >= STEP_CALC)
		{
			for(int i = 0; i < (int)joints.size(); i++)
			{
				joints.at(i)->setJointPosition((double)sum_next.at(i)/STEP_CALC);
				sum_next.at(i) = 0;
			}

			cm700->moveAll();
			simfile->addFileJointsPosition(joints, 2.0*sim_time);

			step = 0;
		}

		usleep((int)(2.0*DELTA_T*1000000.0));
		sim_time += DELTA_T;
	}

	simfile->closeJointsPositionFile();

	delete(hyperneat);
	
	return(0);

}

#endif