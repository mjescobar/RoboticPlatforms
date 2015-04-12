#ifndef PRUEBA_CPP
#define PRUEBA_CPP

#include "example.hpp"

int main(int argc, char* argv[])
{	
	CM700 * cm700 = new CM700(9,2);
	Joint * joint = new Joint(cm700,(char*)"AX", 5, 90*RAD, -90*RAD, (char*)"DEG",0);

	while (1)
	{
		double position;
		cout << "Ingrese un valor para el angulo del motor en DEG ó 999 para terminar:" << endl;
		cin >> position;

		if(position == 999) break;
		else
		{	
			clog << "moving to position: " << position << endl;
			joint->setJointPosition(position);
			cm700->moveAll();
			sleep(2);
			cm700->refreshAll();
			clog << "joint_pos: " << joint->getJointCurrentPosition() << endl;
		} 
	}
	
	delete(cm700);
	delete(joint);

	return(0);
}

#endif
