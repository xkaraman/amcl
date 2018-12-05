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
    m_Uniform = true;
}

RobotStateDistribution::RobotStateDistribution(
		std::shared_ptr<MapModel> map) {
	m_Map = map->getMap();
    m_RNG = new libPF::CRandomNumberGenerator();
	double zmin,zmax;
	m_Map->getMetricMin(m_XMin,m_YMin,zmin);
	m_Map->getMetricMax(m_XMax,m_YMax,zmax);
	m_ThMin = -M_PI;
	m_ThMax = M_PI;
	m_Uniform = true;
}
RobotStateDistribution::RobotStateDistribution(float xInit, float yInit,
		float yawInit) {
	m_XMin = xInit;
	m_YMin = yInit;
	m_ThMin = yawInit;
    m_RNG = new libPF::CRandomNumberGenerator();
    m_Uniform = false;
}

RobotStateDistribution::~RobotStateDistribution() {
	// TODO Auto-generated destructor stub
	delete m_RNG;
}

void RobotStateDistribution::setUniform(bool uniform) {
	m_Uniform = uniform;
}

const RobotState RobotStateDistribution::draw() const{
	RobotState state;

	if(m_Uniform){
		state.setXPos(m_RNG->getUniform(m_XMin,m_XMax));
		state.setYPos(m_RNG->getUniform(m_YMin,m_YMax));
		state.setTheta(m_RNG->getUniform(m_ThMin, m_ThMax));
	} else {
		state.setXPos(m_XMin + m_RNG->getGaussian(m_XStd));
		state.setYPos(m_YMin + m_RNG->getGaussian(m_YStd));
		state.setTheta(m_ThMin + m_RNG->getGaussian(m_YawStd));
	}
//	double x,y,theta;
//	octomap::ColorOcTreeNode* node;
//	bool unknown = true;
//	do {
//		x = m_RNG->getUniform(m_XMin,m_XMax);
//		y = m_RNG->getUniform(m_YMin,m_YMax);
//		theta = m_RNG->getUniform(m_ThMin,m_ThMax);
//		node = m_Map->search(x,y,0.0);
//		if (node){
//			unknown = false;
//		}
//	} while ( unknown);
//
//	state.setXPos(x);
//	state.setYPos(y);
//	state.setTheta(theta);

	return state;
}

void RobotStateDistribution::setStdDev(float xStd, float yStd, float yawStd) {
	m_XStd = xStd;
	m_YStd = yStd;
	m_YawStd = yawStd;
}
