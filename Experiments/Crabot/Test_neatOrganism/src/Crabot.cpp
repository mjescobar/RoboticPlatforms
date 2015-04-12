#ifndef Crabot_CPP
#define Crabot_CPP

#include "Crabot.hpp"

int main(int argc, char* argv[])
{	
	srand (time(0));

	SimFiles * simfile = new SimFiles(); 
	RobotSimulator * simulator = new RobotSimulator();

	if(argc < 4)
	{
		cerr << "ERROR: The number of arguments is incorrect" << endl;
		return -1;	
	} 
	
	simulator->simStart();
	// ============= VREP INITIALIZATIONS ============= //			

	vector < Joint * > joints;
	Dummy * center_dummy = new Dummy(simulator, (char*)"center");

	double max_angle_limit[] = MAX_ANGLE_LIMIT;
	double min_angle_limit[] = MIN_ANGLE_LIMIT;

	for(int i = 0; i < N_LEGS*GRA_LIB + GRA_LIB_EXT; i++)
	{
		stringstream joint;
		joint << "joint" << i+6;
		joints.push_back(new Joint(simulator ,(char*)joint.str().c_str(), max_angle_limit[i], min_angle_limit[i], (char*)"SCALE"));
	}
	
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
	
	if (simulator->simGetConnectionId() != -1)
	{			
		double sim_time = 0;					
		int step = 0;
		vector < double > sum_next ((int)joints.size(),0.0);

		simfile->openNewJointsPositionFile(0, 0);
		simfile->openNewRobotPositionFile(0, 0);

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

		simulator->simStartSimulation(simx_opmode_oneshot_wait);

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
				simulator->simPauseCommunication(1);

				for(int i = 0; i < (int)joints.size(); i++)
				{
					joints.at(i)->setJointPosition((double)sum_next.at(i)/STEP_CALC);
					sum_next.at(i) = 0;
				}

				simulator->simPauseCommunication(0);

				if(sim_time > TIME_INIT_MEASURING)
				{
					simfile->addFileJointsPosition(joints, sim_time);
					simfile->addFileRobotPosition(center_dummy, sim_time);
				}							

				step = 0;
			}

			usleep((int)(DELTA_T*1000000.0));
			sim_time += DELTA_T;
		}

		simulator->simStopSimulation(simx_opmode_oneshot_wait);

		simfile->closeJointsPositionFile();
		simfile->closeRobotPositionFile();
	}
	
	simulator->simFinish();

	delete(simulator);
	delete(simfile);
	delete(hyperneat);
	
	return(0);

}

#endif