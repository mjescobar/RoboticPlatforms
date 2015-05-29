#ifndef Quadratot_CPP
#define Quadratot_CPP

#include "Quadratot.hpp"

bool fileExist(char * filename){
   
    if (FILE *file = fopen(filename, "r"))
    {
        fclose(file);
        return true;
    } else {
        return false;
    }   
}

int main(int argc, char* argv[])
{	
	srand (time(0));

	SimFiles * simfile = new SimFiles(argv[1]); 
	Fitness * fitness_object = new Fitness();
	RobotSimulator * simulator = new RobotSimulator();

	simfile->openFitnessFile();
	simfile->openInputFitnessFile();

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
		joint << "joint" << i;
		joints.push_back(new Joint(simulator, (char*)joint.str().c_str(), max_angle_limit[i], min_angle_limit[i],(char *)"SCALE"));
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

	stringstream hyperneat_def, user_def, genetic_encoding;
	hyperneat_def << argv[1] << "/hyperneat_def.json";
	user_def << argv[1] << "/user_def";
	genetic_encoding << argv[1] << "/genetic_encoding";	

	HyperNeat * hyperneat = new HyperNeat(pass, next, (char*)hyperneat_def.str().c_str(), (char*)user_def.str().c_str(), (char*)genetic_encoding.str().c_str());

	int generation = atoi(argv[2]);
	int organism_generation = atoi(argv[3]);

	if((generation > hyperneat->cppn_neat->GENERATIONS) || (organism_generation > hyperneat->cppn_neat->POPULATION_MAX))
	{
		clog << "ERROR: The data given are incorrect" << endl;
		return(0);
	}

	vector < int > organism_number;
	vector < char * > organism_filename;

	for(int i = 0; i < hyperneat->cppn_neat->POPULATION_MAX; i++)
	{
		stringstream filename;
		char * c_filename = new char[200];
		filename << argv[1] << "/NEAT_organisms/NEATG" << generation << "P" << i << flush;
		strcpy(c_filename, (char *)filename.str().c_str());

		if(fileExist(c_filename))
		{
			organism_number.push_back(i);
			organism_filename.push_back(c_filename);
		}
	}

	// ================================================ //

	bool repetition = false;
	int organism = 0;

	for(int i = 0; i < organism_generation; i++)
	{
		if(!repetition)
		{
			if(organism_number.size() < 1)
			{
				clog << "Exist no more organisms in this generation" << endl;
				return(0);
			}else
				organism = rand()%organism_number.size();
		}else
			repetition = false;	

		fitness_object->resetValues();

		if (simulator->simGetConnectionId() != -1)
		{	
			double sim_time = 0;					
			int step = 0;
			vector < double > sum_next ((int)joints.size(),0.0);

			simfile->openNewJointsPositionFile(generation, organism_number.at(organism));
			simfile->openNewRobotPositionFile(generation, organism_number.at(organism));

			if(!hyperneat->CreateSubstrateConnections(organism_filename.at(organism)))
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

			clog << endl << "Press intro to start Simulation . . ." << endl;
			cin.get();
			clog << "\tStart Simulation of organism " << organism_number.at(organism) << " of generation " << generation << endl << endl;

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
						fitness_object->measuringValues(joints, center_dummy);

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

			int fitness;

			clog << "Enter '-1' if you want repeat the simulation, or" << endl;
			clog << "Enter the fitness of the simulation between 1 and 100:" << endl;
			cin >> fitness;			
			cin.ignore(256,'\n');

			if(fitness < 0)
			{
				repetition = true;
				i--;
				continue;
			}

			simfile->addFitnessToFile(generation, organism_number.at(organism), fitness);
			simfile->addiInputFitnessToFile(generation, organism_number.at(organism), fitness_object);

			stringstream new_organism_filename;
			new_organism_filename << organism_filename.at(organism) << "_VIEWED";
			rename(organism_filename.at(organism), (char *)new_organism_filename.str().c_str());
			organism_number.erase(organism_number.begin()+organism);
			organism_filename.erase(organism_filename.begin()+organism);

		}else
		{
			clog << "ERROR: VREP connections is broken." << endl;
			return(0);
		}
	}
	

	simulator->simFinish();

	simfile->closeFitnessFile();
	simfile->closeInputFitnessFile();

	delete(simulator);
	delete(simfile);	
	delete(fitness_object);
	delete(hyperneat);
	
	return(0);

}

#endif