#ifndef Argo_CPP
#define Argo_CPP

#include "Argo.hpp"

int cantidadDeVreps;
vector <RobotSimulator * > simulators;
Population * cppn_neat;
char * HyperNEATPath;
int currentGeneration;
int cppnOutputAmount;


void * calcOrganismFitness(void * arg)
{
	int segmento = *((int *)(arg));


	// ============= VREP INITIALIZATIONS ============= //			

	vector < Joint * > joints;
	vector < CollisionObject * > body_parts;
	Dummy * center_dummy = new Dummy(simulators.at(segmento), (char*)"center");

	double max_angle_limit[] = MAX_ANGLE_LIMIT;
	double min_angle_limit[] = MIN_ANGLE_LIMIT;

	for(int i = 0; i < N_LEGS*GRA_LIB + GRA_LIB_EXT; i++)
	{
		stringstream joint;
		joint << "joint" << i << "#";
		joints.push_back(new Joint(simulators.at(segmento), (char*)joint.str().c_str(), max_angle_limit[i], min_angle_limit[i], (char*)"SCALE"));
	}

	for(int i = 0; i < 20; i++)
	{
		stringstream body_part;
		body_part << "Collision" << i << "#";
		body_parts.push_back(new CollisionObject(simulators.at(segmento), (char*)body_part.str().c_str()));
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

	HyperNeat * hyperneat = new HyperNeat(pass, next, HyperNEATPath,cppnOutputAmount);
	// ================================================ //


	Fitness * fitness = new Fitness();

	for(int p = segmento*(cppn_neat->POPULATION_MAX/cantidadDeVreps); p < segmento*(cppn_neat->POPULATION_MAX/cantidadDeVreps)+cppn_neat->POPULATION_MAX/cantidadDeVreps; p++)
	{
		double sim_time = 0;					
		int step = 0;
		bool flag = true;
		stringstream message1, message2;
		vector < double > sum_next ((int)joints.size(),0.0);
		fitness->resetPopulationValues();


		if(!hyperneat->CreateSubstrateConnections( &cppn_neat->organisms.at(p) )   ) continue;

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

		simulators.at(segmento)->simStartSimulation(simx_opmode_oneshot_wait);

		message1 << "Generation " << currentGeneration << " Population " << p;
		simulators.at(segmento)->simAddStatusbarMessage((char*)message1.str().c_str() , simx_opmode_oneshot_wait);

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
				simulators.at(segmento)->simPauseCommunication(1);

				for(int i = 0; i < (int)joints.size(); i++)
				{
					joints.at(i)->setJointPosition((double)sum_next.at(i)/STEP_CALC);
					sum_next.at(i) = 0;
				}

				simulators.at(segmento)->simPauseCommunication(0);

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
				}							

				step = 0;
			}

			usleep((int)(DELTA_T*1000000.0));
			sim_time += DELTA_T;
		}

		simulators.at(segmento)->simStopSimulation(simx_opmode_oneshot_wait);


		if(flag)
		{						
			fitness->calcFitness();

			clog << "======================================  G" << currentGeneration << " P" << p <<endl;
			clog << fitness->getFitnessResults() << endl;



			bool result = (cppn_neat->fitness_champion < fitness->getFitness()) ? true: false;

			if(result) {

				clog << endl << "\tNEW CHAMPION FITNESS\t-->\t" << fitness->getFitness() << endl;

				cppn_neat->fitness_champion = fitness->getFitness();
				cppn_neat->champion = cppn_neat->organisms.at(p);
			}

			cppn_neat->organisms.at(p).fitness = fitness->getFitness();


			simulators.at(segmento)->simAddStatusbarMessage((char*)message2.str().c_str() , simx_opmode_oneshot_wait);
		}
		else
		{	
			cppn_neat->organisms.at(p).fitness = FAILED_FITNESS;
		}
	}

	delete(hyperneat);
	return NULL;
}

class VrepClients{
public:
	VrepClients(string ruta){
		std::ifstream input( ruta.c_str() );
		input >> amountOfClients;

		for (int i = 0; i < amountOfClients; ++i)
		{
			string _ip;
			int _port;
			input >> _ip >> _port;
			ip.push_back(_ip);
			ports.push_back(_port);
			//cout << "ip: " << _ip << "\tport: " << _port << endl;
		}


	};

	int getAmountOfClients(){return amountOfClients;};
	string getIpAt(int place){return ip.at(place);};
	int getPortAt(int place){return ports.at(place);};

private:
	vector <int> ports;
	vector <string> ip;
	int amountOfClients;
};



int main(int argc, char * argv[])
{	
	srand (time(0));
	

	if(argc != 5)
	{
		cerr << "ERROR: The number of arguments is incorrect" << endl;
		return -1;	
	} 


	HyperNEATPath = argv[1];

	VrepClients vrepclients = VrepClients(argv[4]);

	
	int cantidadVreps = vrepclients.getAmountOfClients();
	for (int i = 0; i < cantidadVreps; ++i)
	{
		string ip = vrepclients.getIpAt(i);
		int port = vrepclients.getPortAt(i);
		cout << "ip: " << ip << "\tport: " << port << endl;
	}
	cantidadDeVreps = cantidadVreps;
	char neatname[] = "NEAT";
	char ruta[] = "./NEAT_organisms/";
	cppn_neat = new Population(argv[2],argv[3], neatname, ruta, cantidadVreps);
	

	cppnOutputAmount=cppn_neat->champion.getNEATOutputSize();
	for (int i = 0; i < cantidadVreps; ++i)
	{		
		RobotSimulator * simulator =  new RobotSimulator();
		simulator->simStart(vrepclients.getIpAt(i).c_str(),vrepclients.getPortAt(i));
		simulators.push_back(simulator);
	}
	
	
	
	for(int g = 0; g < cppn_neat->GENERATIONS; g++)
	{
		currentGeneration=g;
		
		pthread_t tid[cantidadVreps];
		for (int i = 0; i < cantidadVreps; ++i)
		{
			int * segm = new int(i);
			pthread_create(&tid[i], NULL, calcOrganismFitness, segm);
		}

		for (int i = 0; i < cantidadVreps; ++i)
		{
			pthread_join(tid[i], NULL);
		}				
				
			
		cppn_neat->print_to_file_currrent_generation();
		cppn_neat->epoch();
	}
	
	clog << endl << "BEST RESULT ------------------------------------" << endl << endl;
	clog << "\t-> " << cppn_neat->fitness_champion << endl << endl;
	
	for (unsigned int i = 0; i < simulators.size(); ++i)
	{
		simulators.at(i)->simFinish();
		delete(simulators.at(i));
	}
	

	

	
	return(0);

}

#endif