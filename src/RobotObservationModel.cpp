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

RobotObservationModel::RobotObservationModel(ros::NodeHandle *nh,
		std::shared_ptr<MapModel> mapModel) :
		libPF::ObservationModel<RobotState>(), m_ZHit(0.8), m_ZShort(0.1), m_ZRand(
				0.05), m_ZMax(0.05), m_SigmaHit(0.2), m_LambdaShort(0.1) {
	m_Map = mapModel->getMap();
	m_BaseToSensorTransform.setIdentity();
	ROS_INFO("Robot Observation Model Created!");
}

RobotObservationModel::~RobotObservationModel() {
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
	globalLaserOrigin = tmp.matrix();


	PointCloud pcTransformed;
	pcl::transformPointCloud(m_observedMeasurement, pcTransformed,
			tmp);

//	// Visualization
//	  printf(  "\nPoint cloud colors :  white  = original point cloud\n"
//	      "                        red  = transformed point cloud\n");
//	  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
//
//	   // Define R,G,B colors for the point cloud
//	  PointCloud::Ptr obs = m_observedMeasurement.makeShared();
//	  PointCloud::Ptr trs = pcTransformed.makeShared();
//	  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (obs, 255, 255, 255);
//	  // We add the point cloud to the viewer and pass the color handler
//	  viewer.addPointCloud(obs, source_cloud_color_handler, "original_cloud");
//
//	  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (trs, 230, 20, 20); // Red
//	  viewer.addPointCloud(trs, transformed_cloud_color_handler, "transformed_cloud");
//
//	  viewer.addCoordinateSystem (1.0, "cloud", 0);
//	  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
//	  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
//	  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
//	  //viewer.setPosition(800, 400); // Setting visualiser window position
//
//	  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
//	    viewer.spinOnce ();
//	  }


	// Loop over all Laser Beams
	PointCloud::const_iterator pcIter = pcTransformed.begin();
	std::vector<float>::const_iterator rangesIter = m_observedRanges.begin();

	double weight = 1.0;
//#pragma omp for
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

		if (m_Map->castRay(originP, direction, end, true, 1.5 * maxRange)) {
			ROS_ASSERT(m_Map->isNodeOccupied(m_Map->search(end)));
			raycastRange = (originP - end).norm();
		}

		float z = obsRange - raycastRange;

		// todo check normalization factors in Probabilistics Robotics page 138
		if(obsRange < maxRange)
			p += m_ZHit * 1/(std::sqrt(2 * M_PI * m_SigmaHit * m_SigmaHit))
						* exp(-(z * z) / (2 * m_SigmaHit * m_SigmaHit));

		if( z < 0)
			p += m_ZShort * m_LambdaShort * exp(-m_LambdaShort * obsRange);

		if( obsRange >= maxRange )
			p += m_ZMax * 1.0;

		if (obsRange < maxRange)
			p += m_ZRand * 1.0 / maxRange;

		ROS_ASSERT(p > 0.0);
		weight *= p ;
	} // END OF LOOP OVER LASER BEAMS

//	ROS_INFO("Weight of Particle is [%f]",weight);
	return weight;
}

void RobotObservationModel::setObservedMeasurements(PointCloud observed,
		std::vector<float> ranges) {
	m_observedMeasurement = observed;
	m_observedRanges = ranges;
}

void RobotObservationModel::setTrueCarState(const RobotState& state) {
//    m_TrueCarState = state;
}

void RobotObservationModel::setMap(std::shared_ptr<octomap::ColorOcTree> map) {
	m_Map = map;
}

void RobotObservationModel::setBaseToSensorTransform(
		tf2::Transform baseToSensor) {
	m_BaseToSensorTransform = baseToSensor;
}
