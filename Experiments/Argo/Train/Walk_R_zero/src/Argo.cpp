#ifndef Argo_CPP
#define Argo_CPP

#include "Argo.hpp"

int cantidadDeVreps;
vector <RobotSimulator * > simulators;
vector < Fitness * > fitnesss;
Population * cppn_neat;
SimFiles * simfile;
char * HyperNEATPath;
int currentGeneration;
int cppnOutputAmount;

vector <HyperNeat *> hyperneats;
vector < vector < double * > > nexts;
vector < vector < double * > > passs;
vector < vector < Joint * > > jointss;
vector < vector < CollisionObject * > > body_partss;
vector < Dummy *> center_dummys;

void * calcOrganismFitness(void * arg)
{
	int segmento = *((int *)(arg));

	for(int p = segmento*(cppn_neat->POPULATION_MAX/cantidadDeVreps); p < segmento*(cppn_neat->POPULATION_MAX/cantidadDeVreps)+cppn_neat->POPULATION_MAX/cantidadDeVreps; p++)
	{
		double sim_time = 0;					
		int step = 0;
		bool flag = true;
		stringstream message1, message2;
		vector < double > sum_next ((int)jointss.at(segmento).size(),0.0);
		fitnesss.at(segmento)->resetPopulationValues();


		if(!hyperneats.at(segmento)->CreateSubstrateConnections( &cppn_neat->organisms.at( p ) )   ) continue;

		for(int i = 0; i < (int)jointss.at(segmento).size(); i++)
		{						
			double joint_pos = jointss.at(segmento).at(i)->getJointInitialPosition();
			jointss.at(segmento).at(i)->setJointInitialPosition();
			*nexts.at(segmento).at(i) = joint_pos;
			*passs.at(segmento).at(i) = joint_pos;
		}

		//Revisar - esto es para evitar un problema de error en lectura de primeros valores.
		center_dummys.at(segmento)->getPosition(-1, NULL);
		center_dummys.at(segmento)->getPosition(-1, NULL);
		center_dummys.at(segmento)->getOrientation(-1, NULL);
		center_dummys.at(segmento)->getOrientation(-1, NULL);
		//

		simulators.at(segmento)->simStartSimulation(simx_opmode_oneshot_wait);

		message1 << "Generation " << currentGeneration << " Population " << p;
		simulators.at(segmento)->simAddStatusbarMessage((char*)message1.str().c_str() , simx_opmode_oneshot_wait);

		while(sim_time < TIME_SIMULATION && flag)
		{						
			for(int i = 0; i < ADITIONAL_HYPERNEAT_INPUTS; i++)
			{
				*passs.at(segmento).at((int)jointss.at(segmento).size()+i) = SIN(sim_time,i);
			}

			hyperneats.at(segmento)->EvaluateSubstrateConnections();

			for(int i = 0; i < (int)jointss.at(segmento).size(); i++)
			{
				sum_next.at(i) = sum_next.at(i) + *nexts.at(segmento).at(i);
				*passs.at(segmento).at(i) = *nexts.at(segmento).at(i);
			}		
			step++;
			
			if(step >= STEP_CALC)
			{
				simulators.at(segmento)->simPauseCommunication(1);

				for(int i = 0; i < (int)jointss.at(segmento).size(); i++)
				{
					jointss.at(segmento).at(i)->setJointPosition((double)sum_next.at(i)/STEP_CALC);
					sum_next.at(i) = 0;
				}

				simulators.at(segmento)->simPauseCommunication(0);

				for(int i = 4; i < (int)body_partss.at(segmento).size(); i++)
				{
					if(body_partss.at(segmento).at(i)->getCollisionState() != 0)
					{
						flag = false;
						break;
					}
				}

				if(sim_time > TIME_INIT_MEASURING)
				{
					fitnesss.at(segmento)->measuringValues(jointss.at(segmento), center_dummys.at(segmento));
				}							

				step = 0;
			}

			usleep((int)(DELTA_T*1000000.0));
			sim_time += DELTA_T;
		}

		simulators.at(segmento)->simStopSimulation(simx_opmode_oneshot_wait);


		if(flag)
		{						
			fitnesss.at(segmento)->calcFitness();

			clog << "======================================  G" << currentGeneration << " P" << p <<endl;
			clog << fitnesss.at(segmento)->getFitnessResults() << endl;



			bool result = (cppn_neat->fitness_champion < fitnesss.at(segmento)->getFitness()) ? true: false;

			if(result) {

				clog << endl << "\tNEW CHAMPION FITNESS\t-->\t" << fitnesss.at(segmento)->getFitness() << endl;

				cppn_neat->fitness_champion = fitnesss.at(segmento)->getFitness();
				cppn_neat->champion = cppn_neat->organisms.at(p);
			}

			cppn_neat->organisms.at(p).fitness = fitnesss.at(segmento)->getFitness();


			simulators.at(segmento)->simAddStatusbarMessage((char*)message2.str().c_str() , simx_opmode_oneshot_wait);
		}
		else
		{	
			cppn_neat->organisms.at(p).fitness = FAILED_FITNESS;
		}
	}

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

	double fitness_champion = 0.0;
	
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
	char ruta[] = "./NEAT_organisms/all/";
	cppn_neat = new Population(argv[2],argv[3], neatname, ruta, cantidadVreps);

	simfile = new SimFiles(); 
	

	cppnOutputAmount=cppn_neat->champion.getNEATOutputSize();
	for (int i = 0; i < cantidadVreps; ++i)
	{		
		RobotSimulator * simulator =  new RobotSimulator();
		simulator->simStart(vrepclients.getIpAt(i).c_str(),vrepclients.getPortAt(i));
		simulators.push_back(simulator);

		Fitness * fitness = new Fitness();
		fitnesss.push_back(fitness);

		// ============= VREP INITIALIZATIONS ============= //			

		vector < Joint * > joints;
		vector < CollisionObject * > body_parts;
		Dummy * center_dummy = new Dummy(simulators.at(i), (char*)"center");

		double max_angle_limit[] = MAX_ANGLE_LIMIT;
		double min_angle_limit[] = MIN_ANGLE_LIMIT;

		for(int k = 0; k < N_LEGS*GRA_LIB + GRA_LIB_EXT; k++)
		{

			stringstream joint;
			joint << "joint" << k << "#";
			joints.push_back(new Joint(simulators.at(i), (char*)joint.str().c_str(), max_angle_limit[k], min_angle_limit[k], (char*)"SCALE"));
		}

		for(int k = 0; k < 17; k++)
		{
			stringstream body_part;
			body_part << "Collision" << k << "#";
			body_parts.push_back(new CollisionObject(simulators.at(i), (char*)body_part.str().c_str()));
		}
		
		// ================================================ //
		// ========== HYPERNEAT INITIALIZATIONS =========== //
		vector < double * > next;
		vector < double * > pass;

		for(int k = 0; k < N_LEGS*GRA_LIB + GRA_LIB_EXT; k++)
		{
			double joint_pos = joints.at(k)->getJointInitialPosition();
			double * next_pos = new double(joint_pos);
			double * pass_pos = new double(joint_pos);
			next.push_back(next_pos);
			pass.push_back(pass_pos);
		}

		for(int k = 0; k < ADITIONAL_HYPERNEAT_INPUTS; k++)
		{
			double * aux_pass = new double(SIN(0,k));
			pass.push_back(aux_pass);
		}

		HyperNeat * hyperneat = new HyperNeat(pass, next, HyperNEATPath,cppnOutputAmount);
		// ================================================ //


		nexts.push_back(next);
		passs.push_back(pass);
		hyperneats.push_back(hyperneat);
		jointss.push_back(joints);
		center_dummys.push_back(center_dummy);
		body_partss.push_back(body_parts);
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

		simfile->addFileFitness(fitnesss, g);

		for (int i = 0; i < (int)fitnesss.size(); i++)
			fitnesss.at(i)->resetGenerationValues();
			
		double g_champion = 0.0;
		double g_i = 0;	

		for (int i = 0; i < (int)cppn_neat->organisms.size(); i++)
		{	
			double aux = cppn_neat->organisms.at(i).fitness;
			if(g_champion < aux)
			{
				g_i = i;
				g_champion = aux;
			}
		}

		stringstream org;
		org << "./NEAT_organisms/champions/G" << g;
		cppn_neat->organisms.at(g_i).save((char*)org.str().c_str());

		cppn_neat->print_to_file_currrent_generation();
		cppn_neat->epoch();	

		if(fitness_champion < g_champion)
		{
			fitness_champion = g_champion;
			cppn_neat->organisms.at(g_i).save((char*)"./NEAT_organisms/champions/champion");
		}
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