/*
 * RobotStateDistribution.h
 *
 *  Created on: Oct 1, 2018
 *      Author: xenakis
 */

#ifndef SRC_ROBOTSTATEDISTRIBUTION_H_
#define SRC_ROBOTSTATEDISTRIBUTION_H_

#include <libPF/StateDistribution.h>
#include <RobotState.h>

namespace libPF {
class RandomNumberGenerationStrategy;
} /* namespace libPF */

class RobotStateDistribution : public libPF::StateDistribution<RobotState>{
public:
	RobotStateDistribution(float xmin, float xmax, float ymin, float ymax,float yawmin, float yawmax);
	virtual ~RobotStateDistribution();

	const RobotState draw() const;
private:
    float m_XMin, m_XMax, m_YMin, m_YMax, m_ThMin,m_ThMax;
    libPF::RandomNumberGenerationStrategy *m_RNG;
};

#endif /* SRC_ROBOTSTATEDISTRIBUTION_H_ */
