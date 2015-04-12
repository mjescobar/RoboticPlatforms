#ifndef DUMMY_HPP
#define DUMMY_HPP

#include "Object.hpp"

/**
 * \namespace ANN_USM
 * \brief Dedicated to artificial intelligence development in Santa María University.
 */
namespace ANN_USM
{
	/**
	 * \class Dummy
	 * \brief The Dummy class is inherited from the Object class and is used to interact with objects in an virtual environment of simulation named VREP or in real robots, more easily and transparently for the user.
	 */
	class Dummy : public Object
	{
		// pointer to 3 values receiving the linearVelocity (vx, vy, vz).
		double * velocity;

	public:

		/**
		 * \brief Constructor with parameters
		 * \param simulator Object of type Robotsimulator used to interact with VREP.
		 * \param name Name of the Dummy object
		 */
		Dummy(RobotSimulator * simulator, char name[]);

		/**
		 * \brief Void constructor
		 */
		Dummy();

		/**
		 * \brief Destructor
		 */
		~Dummy();

		/**
		 * \brief Retrieves the linear velocity of an object.
		 * \param velocity Array of 3 values receiving the linearVelocity (vx, vy, vz). Can be NULL.
		 */
		void getVelocity(double velocity[3]);
	};
}

#endif