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

#include <MapModel.h>

namespace libPF {
class RandomNumberGenerationStrategy;
} /* namespace libPF */

class RobotStateDistribution : public libPF::StateDistribution<RobotState>{
public:
	// Constructor for Uniform use case
	RobotStateDistribution(float xmin, float xmax, float ymin, float ymax,float yawmin, float yawmax);

	// Constructor for Gaussian use
	RobotStateDistribution(float xInit,float yInit,float yawInit);

	// Constructor for Global localization use
	RobotStateDistribution(std::shared_ptr<MapModel> map);

	// Deconstructor
	virtual ~RobotStateDistribution();

	// Set where the draw() function draws from Uniform distribution
	// @param uniform whether to use Uniform distribution, false gaussian
	virtual void setUniform(bool uniform);

	// Set Standard deviations for x,y,yaw positions
	void setStdDev(float xStd, float yStd, float yawStd);

	// See libPF::StateDistribution<RobotState>::draw()
	const RobotState draw() const;
private:
	// X,Y,Yaw(Theta) variables
	// m_XMin,m_YMin,m_ThMin used also as start position for gaussian distribution
    double m_XMin, m_XMax, m_YMin, m_YMax, m_ThMin,m_ThMax;
    double m_XStd, m_YStd, m_YawStd;
    bool m_Uniform;
    std::shared_ptr<octomap::ColorOcTree> m_Map;

    libPF::RandomNumberGenerationStrategy *m_RNG;
};

#endif /* SRC_ROBOTSTATEDISTRIBUTION_H_ */
