#ifndef CALCFUNCTIONS_CPP
#define CALCFUNCTIONS_CPP

#include "CalcFunctions.hpp"

double mean(vector < double > numbers)
{	
	if((int)numbers.size() == 0) return 0.0;

	double average = 0;

	for(int i = 0; i < (int)numbers.size(); i++)
		average += numbers.at(i);
	
	return (double)average/numbers.size();
}

double mean(vector < int > numbers)
{	
	if((int)numbers.size() == 0) return 0.0;

	double average = 0;

	for(int i = 0; i < (int)numbers.size(); i++)
		average += numbers.at(i);
	
	return (double)average/numbers.size();
}

double stdDesviation(vector < double > numbers)
{
	if((int)numbers.size() == 0) return 0.0;

	double average = mean(numbers);
	double sum = 0;

	for(int i = 0; i < (int)numbers.size(); i++)
		sum += pow(numbers.at(i)-average,2);
	
	return sqrt(sum/numbers.size());
}

double stdDesviation(vector < int > numbers)
{
	if((int)numbers.size() == 0) return 0.0;

	double average = mean(numbers);
	double sum = 0;

	for(int i = 0; i < (int)numbers.size(); i++)
		sum += pow(numbers.at(i)-average,2);
	
	return sqrt(sum/numbers.size());
}

#endif