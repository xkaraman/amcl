/*
 * CPPRandomNumberGenerator.cpp
 *
 *  Created on: Oct 17, 2018
 *      Author: xenakis
 */

#include <CPPRandomNumberGenerator.h>

CPPRandomNumberGenerator::CPPRandomNumberGenerator()
//:		m_RandomEngine(), m_NormalDistribution(), m_UniformDistribution()
{
	// TODO Auto-generated constructor stub
//	seeder = std::random_device();
	generator = new std::mt19937();
}

CPPRandomNumberGenerator::~CPPRandomNumberGenerator() {
	// TODO Auto-generated destructor stub
}

double CPPRandomNumberGenerator::getGaussian(double standardDeviation) const {
	if (standardDeviation < 0) {
		standardDeviation = -standardDeviation;
	}

//	m_NormalDistribution = std::normal_distribution<double>(0,standardDeviation);
//	std::mt19937 generator = std::mt19937();
	std::normal_distribution<double> normalDist(0.0,standardDeviation);
	double res = normalDist(*generator);
	return res;
}

double CPPRandomNumberGenerator::getUniform(double min, double max) const {
//	m_UniformDistribution = std::uniform_real_distribution<double>(min,max);
//	std::mt19937 generator = std::mt19937();
	std::uniform_real_distribution<double> uniDist(min,max);
	return uniDist(*generator);
}
