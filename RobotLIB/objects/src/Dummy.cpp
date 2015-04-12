#ifndef DUMMY_CPP
#define DUMMY_CPP

#include "Dummy.hpp"
using namespace ANN_USM;

Dummy::Dummy(RobotSimulator * simulator, char name[]) : Object(simulator, name)
{
	velocity = new double[3];
}

Dummy::Dummy()
{

}

Dummy::~Dummy()
{

}

void Dummy::getVelocity(double velocity[3])
{
	simulator->simGetObjectVelocity(handle, this->velocity, simx_opmode_oneshot);
	
	if (velocity != NULL)
		for (int i = 0; i < 3; i++)
			velocity[i] = this->velocity[i];		
}

#endif

