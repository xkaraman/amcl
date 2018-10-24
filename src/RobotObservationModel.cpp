#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <octomap/ColorOcTree.h>
#include <octomap/octomap_types.h>

#include <geometry_msgs/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>

#include <RobotObservationModel.h>

#include <memory>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>

#include <libColor/src/color/color.hpp>

RobotObservationModel::RobotObservationModel(ros::NodeHandle *nh,
		std::shared_ptr<MapModel> mapModel) :
		libPF::ObservationModel<RobotState>(), m_ZHit(0.8), m_ZShort(0.1), m_ZRand(
				0.05), m_ZMax(0.05), m_SigmaHit(0.2), m_LambdaShort(0.1),
				m_RGB(false){
	m_Map = mapModel->getMap();
	m_BaseToSensorTransform.setIdentity();
	ROS_INFO("Robot Observation Model Created!");
}

RobotObservationModel::~RobotObservationModel() {
}

double computeWeight(PointCloudRGB measurement, PointCloudRGB map) {
	pcl::PointCloud<PointRGBT>::Ptr model(
			new pcl::PointCloud<PointRGBT>(measurement));
	pcl::PointCloud<PointRGBT>::Ptr model_keypoints(
			new pcl::PointCloud<PointRGBT>(measurement));
	pcl::PointCloud<PointRGBT>::Ptr mapCloud(
			new pcl::PointCloud<PointRGBT>(map));
	pcl::PointCloud<PointRGBT>::Ptr map_keypoints(
			new pcl::PointCloud<PointRGBT>(map));
	pcl::PointCloud<pcl::Normal>::Ptr model_normals(
			new pcl::PointCloud<pcl::Normal>());
	pcl::PointCloud<pcl::Normal>::Ptr map_normals(
			new pcl::PointCloud<pcl::Normal>());
	pcl::PointCloud<pcl::SHOT1344>::Ptr model_descriptors(
			new pcl::PointCloud<pcl::SHOT1344>());
	pcl::PointCloud<pcl::SHOT1344>::Ptr map_descriptors(
			new pcl::PointCloud<pcl::SHOT1344>());

	//
	//  Compute Normals
	//
	pcl::NormalEstimationOMP<PointRGBT, pcl::Normal> norm_est;
	norm_est.setRadiusSearch(0.5);
	norm_est.setInputCloud(model);
	norm_est.compute(*model_normals);

	norm_est.setInputCloud(mapCloud);
	norm_est.compute(*map_normals);

	//SHOT1344
	//  Compute Descriptor for keypoints
	//
	pcl::SHOTColorEstimationOMP<PointRGBT, pcl::Normal, pcl::SHOT1344> descr_est;
	descr_est.setRadiusSearch(0.02);

	descr_est.setInputCloud(model_keypoints);
	descr_est.setInputNormals(model_normals);
	descr_est.setSearchSurface(model);
	descr_est.compute(*model_descriptors);

	descr_est.setInputCloud(map_keypoints);
	descr_est.setInputNormals(map_normals);
	descr_est.setSearchSurface(mapCloud);
	descr_est.compute(*map_descriptors);

	//
	//  Find Model-Scene Correspondences with KdTree
	//
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

	pcl::KdTreeFLANN<pcl::SHOT1344> match_search;
	match_search.setInputCloud(model_descriptors);
	// For each scene keypoint descriptor,
	// find nearest neighbor into the model keypoints descriptor cloud and
	// add it to the correspondences vector.
	for (size_t i = 0; i < map_descriptors->size(); ++i) {
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!pcl_isfinite(map_descriptors->at(i).descriptor[0])) //skipping NaNs
				{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(map_descriptors->at(i),
				1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
				{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i),
					neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size()
			<< std::endl;

}

double RobotObservationModel::measure(const RobotState& state) const {
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

	PointCloudRGB pcTransformed;
	pcl::transformPointCloud(m_observedMeasurementRGB, pcTransformed, tmp);

	PointCloudRGB::const_iterator pcIter = pcTransformed.begin();
	std::vector<float>::const_iterator rangesIter =	m_observedRanges.begin();

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
		if(raycastRange == 0 )
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

		double colorSimilarity;
		if(m_RGB && !colorNode){
			color::rgb<uint8_t> obsColor({pcIter->r,pcIter->g,pcIter->b});
			color::rgb<uint8_t> raycastColor({colorNode->getColor().r,
											  colorNode->getColor().g,
											  colorNode->getColor().b });
			// Less is more similar
			// Greater is not similar
			// Lower the better
			colorSimilarity = color::operation::distance < ::color::constant::distance::CIEDE2000_entity>(obsColor,raycastColor); // @suppress("Invalid arguments") // @suppress("Symbol is not resolved")

			// Domain 0 to +inf
			// Return 0 to 1
			colorSimilarity = std::tanh(colorSimilarity);
			ROS_ASSERT(colorSimilarity >= 0.0);
		}

		ROS_ASSERT(p > 0.0);

		if(!m_RGB){
			weight *= p;
		}
		else {
			if(colorSimilarity == 0.0)
				colorSimilarity = 0.01;
			weight *= p * (1/colorSimilarity);
		}

	}
	return weight;
//
//		PointCloudRGB octoMapPointCloud;
//		// Create PointCloud from Octomap hit Points
//		for (; pcIter != pcTransformed.end(); pcIter++) {
//			double p = 0.0;
//			octomap::point3d direction(pcIter->x, pcIter->y, pcIter->z);
//			direction = direction - originP;
//
//			octomap::point3d end;
//
//			// TODO Set as field Parameter
//			double minRange = 1;
//			double maxRange = 8;
//
//			octomap::ColorOcTreeNode *colorNode;
//
//			if (m_Map->castRay(originP, direction, end, true, 1.5 * maxRange)) {
//				ROS_ASSERT(m_Map->isNodeOccupied(m_Map->search(end)));
//				colorNode = m_Map->search(end);
//				PointRGBT tmp;
//				tmp.x = end.x();
//				tmp.y = end.y();
//				tmp.z = end.z();
//				tmp.r = colorNode->getColor().r;
//				tmp.g = colorNode->getColor().g;
//				tmp.b = colorNode->getColor().b;
//
//				octoMapPointCloud.push_back(tmp);
//			}
//		}
//		computeWeight(pcTransformed,octoMapPointCloud);
//		return weight;
}

void RobotObservationModel::setObservedMeasurements(const PointCloud &observed,
		const std::vector<float> &ranges) {
	m_observedMeasurement = observed;
	pcl::copyPointCloud(m_observedMeasurement,m_observedMeasurementRGB);
	m_observedRanges = ranges;
}

void RobotObservationModel::setObservedMeasurements(const PointCloudRGB &observed,
		const std::vector<float> &ranges) {
	m_observedMeasurementRGB = observed;
	m_observedRanges = ranges;
}

void RobotObservationModel::setTrueCarState(const RobotState& state) {
//    m_TrueCarState = state;
}

void RobotObservationModel::setMap(
		const std::shared_ptr<octomap::ColorOcTree> &map) {
	m_Map = map;
}

void RobotObservationModel::setBaseToSensorTransform(
		const tf2::Transform &baseToSensor) {
	m_BaseToSensorTransform = baseToSensor;
}

void RobotObservationModel::setRGB(const bool &rgb) {
	m_RGB = rgb;
}
