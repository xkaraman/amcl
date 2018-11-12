/*
 * LaserObservationModel.cpp
 *
 *  Created on: Nov 5, 2018
 *      Author: xenakis
 */

#include <LaserObservationModel.h>
#include <ros/ros.h>

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/common/transforms.h>

LaserObservationModel::LaserObservationModel(ros::NodeHandle* nh,
		std::shared_ptr<MapModel> mapModel) :
		libPF::ObservationModel<RobotState>(), m_ZHit(0.8), m_ZShort(0.1), m_ZRand(
				0.05), m_ZMax(0.05), m_SigmaHit(0.2), m_LambdaShort(0.1), m_RGB(
				false) {
	m_Map = mapModel->getMap();
	m_BaseToSensorTransform.setIdentity();
	ROS_INFO("Laser Observation Model Created");
}

LaserObservationModel::~LaserObservationModel() {
}

double LaserObservationModel::measure(RobotState const & state) const {
	// transform current particle's pose to its sensor frame
	tf2::Transform particlePose;
	tf2::convert(state.getPose(), particlePose);

	tf2::Transform globalLaserOriginTf = particlePose * m_BaseToSensorTransform;

	// Raycastin Origin Point
	octomap::point3d originP(globalLaserOriginTf.getOrigin().getX(),
			globalLaserOriginTf.getOrigin().getY(),
			globalLaserOriginTf.getOrigin().getZ());

	// Transform Pointcloud
	Eigen::Matrix4d globalLaserOrigin;

	geometry_msgs::Transform transformMsg;
	transformMsg = tf2::toMsg(globalLaserOriginTf);
	Eigen::Affine3d tmp = tf2::transformToEigen(transformMsg);

	PointCloud pcTransformed;
	pcl::transformPointCloud(m_observedMeasurement, pcTransformed, tmp);

	PointCloud::const_iterator pcIter = pcTransformed.begin();
	std::vector<float>::const_iterator rangesIter = m_observedRanges.begin();

	double weight = 1.0;

#pragma omp parallel  for
	for (; pcIter != pcTransformed.end(); pcIter++, rangesIter++) {
		// Probability for weight
		double p = 0.0;

		float obsRange = *rangesIter;
		float raycastRange;
		octomap::point3d direction(pcIter->x, pcIter->y, pcIter->z);
		direction = direction - originP;

		octomap::point3d end;

		// TODO Set as field Parameter
		double minRange = 0.5;
		double maxRange = 8;

		octomap::ColorOcTreeNode *colorNode;
		if (m_Map->castRay(originP, direction, end, true, 1.5 * maxRange)) {
			ROS_ASSERT(m_Map->isNodeOccupied(m_Map->search(end)));
			colorNode = m_Map->search(end);
			raycastRange = (originP - end).norm();
		}

		// Particle in occupied space(??)
		if (raycastRange == 0)
			continue;

		float z = obsRange - raycastRange;

		// todo check normalization factors in Probabilistics Robotics page 138
		if (obsRange < maxRange)
			p += m_ZHit * 1 / (std::sqrt(2 * M_PI * m_SigmaHit * m_SigmaHit))
					* exp(-(z * z) / (2 * m_SigmaHit * m_SigmaHit));

		if (z < 0)
			p += m_ZShort * m_LambdaShort * exp(-m_LambdaShort * obsRange);

		if (obsRange >= maxRange)
			p += m_ZMax * 1.0;

		if (obsRange < maxRange)
			p += m_ZRand * 1.0 / maxRange;

		ROS_ASSERT(p > 0.0);
		weight *= p ;

	}

	return weight;

}

void LaserObservationModel::setMap(
		std::shared_ptr<octomap::ColorOcTree> const & map) {
	m_Map = map;
}

void LaserObservationModel::setBaseToSensorTransform(
		tf2::Transform const & baseToSensor) {
	m_BaseToSensorTransform = baseToSensor;
}

void LaserObservationModel::setObservedMeasurements(PointCloud const & observed,
		std::vector<float> const & ranges) {
	m_observedMeasurement = observed;
	m_observedRanges = ranges;
}
