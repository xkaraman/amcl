/*
 * CPPRandomNumberGenerator.cpp
 *
 *  Created on: Oct 17, 2018
 *      Author: xenakis
 */

#include <CPPRandomNumberGenerator.h>

CPPRandomNumberGenerator::CPPRandomNumberGenerator() :
		m_RandomEngine(), m_NormalDistribution(), m_UniformDistribution() {
	// TODO Auto-generated constructor stub

}

CPPRandomNumberGenerator::~CPPRandomNumberGenerator() {
	// TODO Auto-generated destructor stub
}

double CPPRandomNumberGenerator::getGaussian(double standardDeviation) const {
	if (standardDeviation < 0) {
		standardDeviation = -standardDeviation;
	}

	m_NormalDistribution = std::normal_distribution<double>(0,standardDeviation);
	return m_NormalDistribution(m_RandomEngine);
}

double CPPRandomNumberGenerator::getUniform(double min, double max) const {
	m_UniformDistribution = std::uniform_real_distribution<double>(min,max);
	return m_UniformDistribution(m_RandomEngine);
}
