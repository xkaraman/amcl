/*
 * CPPRandomNumberGenerator.h
 *
 *  Created on: Oct 17, 2018
 *      Author: xenakis
 */

#ifndef SRC_CPPRANDOMNUMBERGENERATOR_H_
#define SRC_CPPRANDOMNUMBERGENERATOR_H_

#include <libPF/RandomNumberGenerationStrategy.h>
#include <random>

class CPPRandomNumberGenerator : public libPF::RandomNumberGenerationStrategy {
public:
	CPPRandomNumberGenerator();

	virtual ~CPPRandomNumberGenerator();

    double getGaussian(double standardDeviation) const;

    double getUniform(double min = 0.0, double max = 1.0) const;

private:
    std::mt19937 m_RandomEngine;
    std::normal_distribution<double> m_NormalDistribution;
    std::uniform_real_distribution<double> m_UniformDistribution;
};

#endif /* SRC_CPPRANDOMNUMBERGENERATOR_H_ */
