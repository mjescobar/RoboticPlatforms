#ifndef Argo_CPP
#define Argo_CPP

#include "Argo.hpp"

int main(int argc, char* argv[])
{	
	srand (time(0));

	SimFiles * simfile = new SimFiles(); 
	Fitness * fitness = new Fitness();
	RobotSimulator * simulator = new RobotSimulator();

	if(argc < 4)
	{
		cerr << "ERROR: The number of arguments is incorrect" << endl;
		return -1;	
	} 

	simulator->simStart();
	// ============= VREP INITIALIZATIONS ============= //			

	vector < Joint * > joints;
	vector < CollisionObject * > body_parts;
	Dummy * center_dummy = new Dummy(simulator, (char*)"center");

	double max_angle_limit[] = MAX_ANGLE_LIMIT;
	double min_angle_limit[] = MIN_ANGLE_LIMIT;

	for(int i = 0; i < N_LEGS*GRA_LIB + GRA_LIB_EXT; i++)
	{
		stringstream joint;
		joint << "joint" << i << "#";
		joints.push_back(new Joint(simulator, (char*)joint.str().c_str(), max_angle_limit[i], min_angle_limit[i], (char*)"SCALE"));
	}

	for(int i = 0; i < 20; i++)
	{
		stringstream body_part;
		body_part << "Collision" << i << "#";
		body_parts.push_back(new CollisionObject(simulator, (char*)body_part.str().c_str()));
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
		for(int g = 0; g < hyperneat->cppn_neat->GENERATIONS; g++)
		{
			fitness->resetGenerationValues();

			for(int p = 0; p < hyperneat->cppn_neat->POPULATION_MAX; p++)
			{
				double sim_time = 0;					
				int step = 0;
				bool flag = true;
				stringstream message1, message2;
				vector < double > sum_next ((int)joints.size(),0.0);

				fitness->resetPopulationValues();

				simfile->openNewJointsPositionFile(g, p);
				simfile->openNewRobotPositionFile(g, p);

				if(!hyperneat->CreateSubstrateConnections(p)) continue;

				for(int i = 0; i < (int)joints.size(); i++)
				{						
					double joint_pos = joints.at(i)->getJointInitialPosition();
					joints.at(i)->setJointInitialPosition();
					*next.at(i) = joint_pos;
					*pass.at(i) = joint_pos;
				}

				//Revisar - esto es para evitar un problema de error en lectura de primeros valores.
				center_dummy->getPosition(-1, NULL);
				center_dummy->getPosition(-1, NULL);
				center_dummy->getOrientation(-1, NULL);
				center_dummy->getOrientation(-1, NULL);
				//

				simulator->simStartSimulation(simx_opmode_oneshot_wait);

				message1 << "Generation " << g << " Population " << p;
				simulator->simAddStatusbarMessage((char*)message1.str().c_str() , simx_opmode_oneshot_wait);

				while(sim_time < TIME_SIMULATION && flag)
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

						for(int i = 4; i < (int)body_parts.size(); i++)
						{
							if(body_parts.at(i)->getCollisionState() != 0)
							{
								flag = false;
								break;
							}
						}

						if(sim_time > TIME_INIT_MEASURING)
						{
							fitness->measuringValues(joints, center_dummy);

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

				if(flag)
				{						
					fitness->calcFitness();

					clog << "======================================  G" << g << " P" << p <<endl;
					clog << fitness->getFitnessResults() << endl;

					simfile->addFileResults(fitness, g, p);		

					hyperneat->HyperNeatFitness(fitness->getFitness(), p);

					message2 << "FITNESS : " << fitness->getFitness();
					simulator->simAddStatusbarMessage((char*)message2.str().c_str() , simx_opmode_oneshot_wait);
				}
				else
				{	
					hyperneat->HyperNeatFitness(FAILED_FITNESS, p);
				}
			}				
			hyperneat->HyperNeatEvolve();
			simfile->addFileFitness(fitness, g);
			simfile->addFileFrecuency(fitness, g);
		}
	}

	clog << endl << "BEST RESULT ------------------------------------" << endl << endl;
	clog << "\t-> " << hyperneat->cppn_neat->fitness_champion << endl << endl;
	
	simfile->addFileResults(hyperneat->cppn_neat->fitness_champion);
	
	simulator->simFinish();

	delete(simulator);
	delete(simfile);
	delete(fitness);
	delete(hyperneat);
	
	return(0);

}

#endif