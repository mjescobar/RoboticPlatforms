#ifndef PRUEBA_CPP
#define PRUEBA_CPP

#include "example.hpp"

int main(int argc, char* argv[])
{	
	RobotSimulator * simulator = new RobotSimulator();
	simulator->simStart();

	Joint * joint = new Joint(simulator,(char*)"joint7",90*RAD,-90*RAD, (char*)"DEG");

	simulator->simStartSimulation(simx_opmode_oneshot_wait);

	while (simulator->simGetConnectionId() != -1)
	{
		double position;
		cout << "Ingrese un valor para el angulo del motor en DEG ó 999 para terminar:" << endl;
		cin >> position;

		if(position == 999) break;
		else
		{				
			double * pos = new double[3];
			double * ori = new double[3];
			joint->getPosition(-1, pos);
			joint->getOrientation(-1, ori);
			clog << "pos: " << pos[0] << " - " << pos[1] << " - " << pos[2] << endl;
			clog << "ori: " << ori[0] << " - " << ori[1] << " - " << ori[2] << endl;
			clog << "moving to position: " << position << endl;
			joint->setJointPosition(position);
			sleep(2);
			clog << "joint_pos: " << joint->getJointCurrentPosition() << endl;
		} 
	}

	simulator->simStopSimulation(simx_opmode_oneshot_wait);	

	simulator->simFinish();

	delete(simulator);
	delete(joint);
	
	return(0);
}

#endif
