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

    virtual double getGaussian(double standardDeviation) const;

    virtual double getUniform(double min = 0.0, double max = 1.0) const;

private:
//    std::mt19937 m_RandomEngine;
//    std::normal_distribution<double> m_NormalDistribution;
//    std::uniform_real_distribution<double> m_UniformDistribution;

    std::random_device seeder;
    std::mt19937 *generator;
    std::normal_distribution<double> distribution;
};

#endif /* SRC_CPPRANDOMNUMBERGENERATOR_H_ */
