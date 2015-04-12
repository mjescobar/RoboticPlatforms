#ifndef TestQuadratot_HPP
#define TestQuadratot_HPP

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <cmath>
#include <time.h>
#include <sys/time.h>
#include <fstream>
#include "Joint.hpp"
#include "Dummy.hpp"
#include "Object.hpp"
#include "RobotSimulator.hpp"
#include "Simulation.hpp"
#include "HYPERNEAT_FUNCTIONS.hpp"

using namespace std;

#define WAVE_FRECUENCY 0.64
#define SIN(X,PHI) (double)sin(2.0*M_PI*WAVE_FRECUENCY*X + PHI*M_PI/2.0)

#endif