/*
 * RobotStateDistribution.cpp
 *
 *  Created on: Oct 1, 2018
 *      Author: xenakis
 */

#include <libPF/CRandomNumberGenerator.h>
#include <RobotStateDistribution.h>
#include <cmath>

RobotStateDistribution::RobotStateDistribution(float xmin, float xmax, float ymin, float ymax,float yawmin, float yawmax) :
			m_XMin(xmin),
			m_XMax(xmax),
			m_YMin(ymin),
			m_YMax(ymax),
			m_ThMin(yawmin),
			m_ThMax(yawmax)
{
	// TODO Auto-generated constructor stub
    m_RNG = new libPF::CRandomNumberGenerator();
}

RobotStateDistribution::~RobotStateDistribution() {
	// TODO Auto-generated destructor stub
	delete m_RNG;
}

const RobotState RobotStateDistribution::draw() const{
	RobotState state;
	state.setXPos(m_RNG->getUniform(m_XMin,m_XMax));
	state.setYPos(m_RNG->getUniform(m_YMin,m_YMax));
	state.setTheta(m_RNG->getUniform(m_ThMin, m_ThMax));

	return state;
}
