#include "TestQuadratot.hpp"

using namespace std;
using namespace ANN_USM;

void calculeMove(vector < double * > pass, vector < double * > next)
{
	*next.at(0) = QUADRATOT_0(*pass.at(0), *pass.at(1), *pass.at(2), *pass.at(3), *pass.at(4), *pass.at(5), *pass.at(6), *pass.at(7), *pass.at(8), *pass.at(9), *pass.at(10));
	*next.at(1) = QUADRATOT_1(*pass.at(0), *pass.at(1), *pass.at(2), *pass.at(3), *pass.at(4), *pass.at(5), *pass.at(6), *pass.at(7), *pass.at(8), *pass.at(9), *pass.at(10));
	*next.at(2) = QUADRATOT_2(*pass.at(0), *pass.at(1), *pass.at(2), *pass.at(3), *pass.at(4), *pass.at(5), *pass.at(6), *pass.at(7), *pass.at(8), *pass.at(9), *pass.at(10));
	*next.at(3) = QUADRATOT_3(*pass.at(0), *pass.at(1), *pass.at(2), *pass.at(3), *pass.at(4), *pass.at(5), *pass.at(6), *pass.at(7), *pass.at(8), *pass.at(9), *pass.at(10));
	*next.at(4) = QUADRATOT_4(*pass.at(0), *pass.at(1), *pass.at(2), *pass.at(3), *pass.at(4), *pass.at(5), *pass.at(6), *pass.at(7), *pass.at(8), *pass.at(9), *pass.at(10));
	*next.at(5) = QUADRATOT_5(*pass.at(0), *pass.at(1), *pass.at(2), *pass.at(3), *pass.at(4), *pass.at(5), *pass.at(6), *pass.at(7), *pass.at(8), *pass.at(9), *pass.at(10));
	*next.at(6) = QUADRATOT_6(*pass.at(0), *pass.at(1), *pass.at(2), *pass.at(3), *pass.at(4), *pass.at(5), *pass.at(6), *pass.at(7), *pass.at(8), *pass.at(9), *pass.at(10));
	*next.at(7) = QUADRATOT_7(*pass.at(0), *pass.at(1), *pass.at(2), *pass.at(3), *pass.at(4), *pass.at(5), *pass.at(6), *pass.at(7), *pass.at(8), *pass.at(9), *pass.at(10));
	*next.at(8) = QUADRATOT_8(*pass.at(0), *pass.at(1), *pass.at(2), *pass.at(3), *pass.at(4), *pass.at(5), *pass.at(6), *pass.at(7), *pass.at(8), *pass.at(9), *pass.at(10));
}

int main(int argc,char* argv[])
{
	srand (time(0));

	RobotSimulator * simulator = new RobotSimulator();

	simulator->simStart();

	// ============= VREP INITIALIZATIONS ============= //			

	vector < Joint * > joints;

	double max_angle_limit[] = MAX_ANGLE_LIMIT;
	double min_angle_limit[] = MIN_ANGLE_LIMIT;

	for(int i = 0; i < N_LEGS*GRA_LIB + GRA_LIB_EXT; i++)
	{
		stringstream joint;
		joint << "joint" << i;
		joints.push_back(new Joint(simulator, (char*)joint.str().c_str(), max_angle_limit[i], min_angle_limit[i]));
	}

	// ================================================ //

	// ========== HYPERNEAT INITIALIZATIONS =========== //

	vector < double * > next;
	vector < double * > pass;

	for(int i = 0; i < N_LEGS*GRA_LIB + GRA_LIB_EXT; i++)
	{
		double joint_pos = joints.at(i)->getJointInitialPosition((char *)"SCALE");

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

	// ================================================ //

	if (simulator->clientIdStatus())
	{
		while (simulator->simGetConnectionId() != -1)
		{								
			double sim_time = 0;
			int step = 0;
			vector < double > sum_next ((int)joints.size(),0.0);

			simulator->simStartSimulation(simx_opmode_oneshot_wait);

			usleep(100000);

			while(sim_time < TIME_SIMULATION)
			{						
				for(int i = 0; i < ADITIONAL_HYPERNEAT_INPUTS; i++)
				{
					*pass.at((int)joints.size()+i) = SIN(sim_time,i);
				}

				calculeMove(pass, next);

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
						joints.at(i)->setJointPosition((double)sum_next.at(i)/STEP_CALC,(char *)"SCALE");
						sum_next.at(i) = 0;			
					}
	
					simulator->simPauseCommunication(0);
					
					step = 0;
				}

				usleep((int)(DELTA_T*1000000.0));
				sim_time += DELTA_T;
			}

			simulator->simStopSimulation(simx_opmode_oneshot_wait);

			break;
		}
	}

	simulator->simFinish();
	delete(simulator);
	return(0);
}



