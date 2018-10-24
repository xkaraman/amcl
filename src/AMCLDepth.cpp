/*
 * AMCLDepth.cpp
 *
 *  Created on: Sep 27, 2018
 *      Author: xenakis
 */

#include <AMCLDepth.h>
//#include <boost/bind/arg.hpp>
//#include <boost/bind/bind.hpp>
//#include <boost/bind/bind_mf_cc.hpp>
//#include <boost/bind/placeholders.hpp>
#include <boost/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
//#include <libPF/ParticleFilter.hxx>
#include <message_filters/connection.h>
#include <message_filters/simple_filter.h>
#include <MapModel.h>
#include <ros/duration.h>
//#include <ros/forwards.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>
#include <RobotMovementModel.h>
#include <RobotObservationModel.h>
#include <RobotStateDistribution.h>
#include <CPPRandomNumberGenerator.h>

#include <std_msgs/Header.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
//#include <string>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

AMCLDepth::AMCLDepth() :
		m_NumberOfParticles(150), m_MapFrameID("map"), m_OdomFrameID("odom"), m_BaseFrameID(
				"base_footprint"), m_ReceivedSensorData(false), m_FirstRun(
				true), m_TFBuffer(ros::Duration(60), false) {
	// Models Used in Particle Filters
	m_MapModel = std::shared_ptr<MapModel>(new OccupancyMap(&m_NH));
//	m_MotionModel = std::shared_ptr<libPF::MovementModel<RobotState> >(new RobotMovementModel(&m_NH,&m_TFBuffer,"odom","map"));
//	m_ObservationModel = std::shared_ptr<libPF::ObservationModel<RobotState> >(new RobotObservationModel());
//	m_MapModel = new OccupancyMap(&m_NH);

	m_MotionModel = new RobotMovementModel(&m_NH, &m_TFBuffer, m_OdomFrameID,
			m_BaseFrameID);
	m_ObservationModel = new RobotObservationModel(&m_NH, m_MapModel);

	m_ParticleFilter = new libPF::ParticleFilter<RobotState>(
			m_NumberOfParticles, m_ObservationModel, m_MotionModel);

	// Initialize TF Buffer,Listeners, Broadcasters
	m_TFBuffer.clear();
	m_TFListener = new tf2_ros::TransformListener(m_TFBuffer);
	m_TFBroadcaster = new tf2_ros::TransformBroadcaster();

	// Create map->odom transform and set it to identity transform
	m_LatestTransform.setIdentity();

	// Initialize PoseArray for Visualization in RVIZ
	m_PoseArray.poses.resize(m_NumberOfParticles);
	m_PoseArray.header.frame_id = m_MapFrameID;

	// Advertise Publishers
	m_PoseArrayPub = m_NH.advertise<geometry_msgs::PoseArray>(
			"amcl_depth/particlecloud", 10);
	m_DepthFilteredPub = m_NH.advertise<PointCloud>("amcl_depth/depthfiltered",
			20);

	// Advertise Services and Initialize Subscribers
	m_GlobalLocService = m_NH.advertiseService("amcl_depth/global_localization",
			&AMCLDepth::globalLocalizationCallback, this);

	m_LaserScanSub = new message_filters::Subscriber<sensor_msgs::LaserScan>(
			m_NH, "/scan", 100);
	m_LaserScanFilter = new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(
			*m_LaserScanSub, m_TFBuffer, m_OdomFrameID, 100, m_NH);
	m_LaserScanFilter->registerCallback(boost::bind(&AMCLDepth::laserCallback,this,_1));

	m_DepthScanSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(
			m_NH, "/camera/depth/points", 100);
	m_DepthScanFilter = new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(
			*m_DepthScanSub, m_TFBuffer, m_OdomFrameID, 100, m_NH);
//	m_DepthScanFilter->registerCallback(boost::bind(&AMCLDepth::depthCallback,this,_1));

	m_RGBScanSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(m_NH,
			"/rgb/points", 100);
	m_RGBScanFilter = new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*m_RGBScanSub,
			m_TFBuffer, m_OdomFrameID, 100, m_NH);
	m_RGBScanFilter->registerCallback(
			boost::bind(&AMCLDepth::rgbCallback, this, _1));

	m_InitPoseSub = new message_filters::Subscriber<
			geometry_msgs::PoseWithCovarianceStamped>(m_NH, "initialpose", 2);
	m_InitPoseFilter = new tf2_ros::MessageFilter<
			geometry_msgs::PoseWithCovarianceStamped>(*m_InitPoseSub,
			m_TFBuffer, m_MapFrameID, 2, m_NH);
	m_InitPoseFilter->registerCallback(
			boost::bind(&AMCLDepth::initPoseCallback, this, _1));

	// Timer for resending the latest transform
	m_LatestTransformTimer = m_NH.createTimer(ros::Duration(0.1),
			&AMCLDepth::latestTimerCallback, this);
//	ROS_INFO("Using timer with a period of %4f s", mTimerPeriod);

	ROS_INFO("Init Ready with %d particles.", m_NumberOfParticles);
}

AMCLDepth::~AMCLDepth() {
	// TODO Auto-generated destructor stub
}

void AMCLDepth::laserCallback(sensor_msgs::LaserScanConstPtr const &msg) {
	ROS_DEBUG("Received Laser Messages at Time: %f", msg->header.stamp.toSec());
	if (!m_Initialized) {
		ROS_WARN("Localization not initialized yet, skipping LaserScan input"
				"\n (1) Use RVIZ 2D Pose Estimation button or"
				"\n (2) Publish to /initialpose topic or"
				"\n (3) Start Global Localization Service");
		return;
	}

	double timediff = (msg->header.stamp - m_LastLaserTime).toSec();
	if (m_ReceivedSensorData && timediff < 0) {
		ROS_WARN(
				"Ignoring received laser data that is %f s older than previous data!",
				timediff);
		return;
	}

	geometry_msgs::PoseStamped odomPose;
	// Check if Odometry is available otherwise skip scan
	if (!m_MotionModel->lookupOdomPose(msg->header.stamp, odomPose)) {
		ROS_WARN("EXITING LASER CALLBACK");
		return;
	}

	if (!m_FirstRun) {
		double dt = (odomPose.header.stamp
				- m_MotionModel->getLastOdomPose().header.stamp).toSec();
		m_LaserIntegrated = false;
		if (!m_ReceivedSensorData || isAboveMotionThreshold(odomPose)) {
			PointCloud pcFiltered;
			std::vector<float> laserRanges;
			prepareLaserPointCloud(msg, pcFiltered, laserRanges);

			ros::Time t = msg->header.stamp;
			geometry_msgs::TransformStamped sensorToBase;
			if (!m_MotionModel->lookupTargetToBaseTransform(
					pcFiltered.header.frame_id, t, sensorToBase)) {
				return;
			}
			tf2::Transform baseToSensor;
			tf2::convert(sensorToBase.transform, baseToSensor);
			baseToSensor = baseToSensor.inverse();

			// TODO Publish filtered PointCloud

			m_ObservationModel->setBaseToSensorTransform(baseToSensor);
			m_ObservationModel->setObservedMeasurements(pcFiltered,
					laserRanges);

			ros::Time past = ros::Time::now();
//			ROS_INFO("Laser filter start in %f s", past.toSec());
			m_ParticleFilter->filter(dt);
			double tdiff = (ros::Time::now() - past).toSec();
			ROS_INFO("Laser filter done in %f s", tdiff);

			m_LaserIntegrated = true;
			if (m_LaserIntegrated && m_DepthIntegrated)
				m_LastLocalizedPose = odomPose.pose;
			m_ReceivedSensorData = true;
		} else {
			m_ParticleFilter->drift(dt);
//			ROS_INFO("laser drift");
		}
	} else {
		m_LastLocalizedPose = odomPose.pose;
	}

	m_MotionModel->setLastOdomPose(odomPose);
	m_FirstRun = false;
	m_LastLaserTime = msg->header.stamp;

	publishPoses(m_LastLaserTime);
//	publishPoseEstimate(msg->header.stamp,sensorIntegrated);
}

void AMCLDepth::initPoseCallback(
		geometry_msgs::PoseWithCovarianceStampedConstPtr const &msg) {

	tf2::Transform pose;
	tf2::convert(msg->pose.pose, pose);

	ROS_INFO("Setting pose near (%.6f): x=%.3f y= %.3f yaw=%.3f",
			ros::Time::now().toSec(), pose.getOrigin().getX(),
			pose.getOrigin().getY(), tf2::getYaw(pose.getRotation()));

	float xmin = pose.getOrigin().getX() - 0.3;
	float xmax = pose.getOrigin().getX() + 0.3;

	float ymin = pose.getOrigin().getY() - 0.3;
	float ymax = pose.getOrigin().getY() + 0.3;

	float yawmin = tf2::getYaw(pose.getRotation()) - M_PI / 5;
	float yawmax = tf2::getYaw(pose.getRotation()) + M_PI / 5;

	RobotStateDistribution distribution(xmin, xmax, ymin, ymax, yawmin, yawmax);
	m_ParticleFilter->drawAllFromDistribution(distribution);
	m_ParticleFilter->resetTimer();
	m_MotionModel->reset();

	m_ReceivedSensorData = false;
	m_Initialized = true;
	m_FirstRun = true;
	publishPoses(msg->header.stamp);
}

void AMCLDepth::latestTimerCallback(const ros::TimerEvent &e) const {
	geometry_msgs::TransformStamped transform;
	transform.header.frame_id = m_MapFrameID;
	transform.header.stamp = ros::Time::now() + ros::Duration(0.1);
	transform.child_frame_id = m_OdomFrameID;
	transform.transform = tf2::toMsg(m_LatestTransform.inverse());
	m_TFBroadcaster->sendTransform(transform);
}

void AMCLDepth::prepareLaserPointCloud(
		sensor_msgs::LaserScanConstPtr const &laser, PointCloud &pc,
		std::vector<float> &ranges) const {
	unsigned int numBeams = laser->ranges.size();
	// Get every n-th scan;
	unsigned int step = 1;

	// Prepare Laser Message
	unsigned int numBeamsSkipped = 0;

	// TODO laserMinRange Parameter
	// TODO laserMaxRange Parameter
	double m_FilterMinRange = 0.05;
	double m_FilterMaxRange = 8;
	double laserMin = std::max(double(laser->range_min), m_FilterMinRange);

	pcl_conversions::toPCL(laser->header, pc.header);

	// Allocate some capacity for vector
	ranges.reserve(50);
	pc.points.reserve(50);

	for (int beamId = 0; beamId < numBeams; beamId += step) {
		double range = laser->ranges[beamId];
		if (range >= laserMin && range <= m_FilterMaxRange) {
			double laserAngle = laser->angle_min
					+ beamId * laser->angle_increment;

			tf2::Transform laserAngleRotation(
					tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), laserAngle));
			tf2::Vector3 laserEndpointTransform(range, 0.0, 0.0);
			tf2::Vector3 pt(laserAngleRotation * laserEndpointTransform);

			pc.points.push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
			ranges.push_back(range);
		} else {
			numBeamsSkipped++;
		}
	}

	pc.height = 1;
	pc.width = pc.points.size();
	pc.is_dense = true;

	// Uniform Sampling
	pcl::UniformSampling<pcl::PointXYZ> uniformSampling;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;
	cloudPtr.reset(new pcl::PointCloud<pcl::PointXYZ>(pc));
	uniformSampling.setInputCloud(cloudPtr);
	// TODO Add as Parameter
	double m_SensorSampleDist = 0.2;
	uniformSampling.setRadiusSearch(m_SensorSampleDist);
	pcl::PointCloud<int> sampledIndices;
	uniformSampling.compute(sampledIndices);

	pcl::copyPointCloud(*cloudPtr, sampledIndices.points, pc);
	// Adjust "ranges" to contain the same points
	std::vector<float> rangesSparse;
	rangesSparse.resize(sampledIndices.size());
	for (unsigned int i = 0; i < rangesSparse.size(); i++) {
		rangesSparse[i] = ranges[sampledIndices.points[i]];
	}
	ranges = rangesSparse;
	ROS_INFO(
			"Laser PointCloud subsampled: %zu from %zu (%u out of valid range)",
			pc.size(), cloudPtr->size(), numBeamsSkipped);
}

void AMCLDepth::publishPoses(const ros::Time &t) {
	m_PoseArray.header.stamp = ros::Time::now();
	if (m_PoseArray.poses.size() != m_ParticleFilter->numParticles()) {
		m_PoseArray.poses.resize(m_ParticleFilter->numParticles());
	}

	for (unsigned i = 0; i < m_ParticleFilter->numParticles(); i++) {
		m_PoseArray.poses[i] = m_ParticleFilter->getState(i).getPose();
	}

	m_PoseArrayPub.publish(m_PoseArray);

	computeBestTransform(t);

	//TODO ADD parameter
	double m_TransformTolerance = 0.1;
	ros::Time transform_expiration = (t + ros::Duration(m_TransformTolerance));
	geometry_msgs::TransformStamped tmp_tf_stamped;
	tmp_tf_stamped.header.frame_id = m_MapFrameID;
	tmp_tf_stamped.header.stamp = transform_expiration;
	tmp_tf_stamped.child_frame_id = m_OdomFrameID;
	tf2::convert(m_LatestTransform.inverse(), tmp_tf_stamped.transform);

	m_TFBroadcaster->sendTransform(tmp_tf_stamped);
}

void AMCLDepth::computeBestTransform(const ros::Time &t) {
	geometry_msgs::PoseStamped odomToMap;
	try {
		tf2::Transform tmp;
		tf2::fromMsg(m_ParticleFilter->getBestState().getPose(), tmp);
		//	tf2::fromMsg(m_ParticleFilter->getMmseEstimate().getPose(), tmp);
		geometry_msgs::PoseStamped tmpStamped;
		tmpStamped.header.frame_id = m_BaseFrameID;
		tmpStamped.header.stamp = t;
		tf2::toMsg(tmp.inverse(), tmpStamped.pose);

		m_TFBuffer.transform(tmpStamped, odomToMap, m_OdomFrameID);
	} catch (tf2::TransformException &e) {
		ROS_DEBUG("Failed to subtract base to odom transform");
		return;
	}

	tf2::convert(odomToMap.pose, m_LatestTransform);
}

void AMCLDepth::depthCallback(sensor_msgs::PointCloud2ConstPtr const &msg) {
	ROS_DEBUG("Received Depth Messages at Time: %f", msg->header.stamp.toSec());
	if (!m_Initialized) {
		ROS_WARN(
				"Localization not initialized yet, skipping DepthPointCloud input"
						"\n (1) Use RVIZ 2D Pose Estimation button or"
						"\n (2) Publish to /initialpose topic or"
						"\n (3) Start Global Localization Service");
		return;
	}

	double timediff = (msg->header.stamp - m_LastDepthCloudTime).toSec();
	if (m_ReceivedSensorData && timediff < 0) {
		ROS_WARN(
				"Ignoring received laser data that is %f s older than previous data!",
				timediff);
		return;
	}

	geometry_msgs::PoseStamped odomPose;
	// Check if Odometry is available otherwise skip scan
	if (!m_MotionModel->lookupOdomPose(msg->header.stamp, odomPose)) {
		ROS_WARN("EXITING DEPTH CLOUD CALLBACK");
		return;
	}

	if (!m_FirstRun) {
		double dt = (odomPose.header.stamp
				- m_MotionModel->getLastOdomPose().header.stamp).toSec();
		m_DepthIntegrated = false;
		if (!m_ReceivedSensorData || isAboveMotionThreshold(odomPose)) {
			PointCloud pcFiltered;
			std::vector<float> laserRanges;
			prepareDepthPointCloud(msg, pcFiltered, laserRanges);
			ros::Time t = msg->header.stamp;
			geometry_msgs::TransformStamped sensorToBase;
			if (!m_MotionModel->lookupTargetToBaseTransform(
					pcFiltered.header.frame_id, t, sensorToBase)) {
				return;
			}
			tf2::Transform baseToSensor;
			tf2::convert(sensorToBase.transform, baseToSensor);
			baseToSensor = baseToSensor.inverse();

			// TODO Publish filtered PointCloud

			m_ObservationModel->setBaseToSensorTransform(baseToSensor);
			m_ObservationModel->setObservedMeasurements(pcFiltered,
					laserRanges);

			ros::Time past = ros::Time::now();
//			ROS_INFO("Depth filter start in %f s", past.toSec());
			m_ParticleFilter->filter(dt);
			double tdiff = (ros::Time::now() - past).toSec();
			ROS_INFO("Depth filter done in %f s", tdiff);

			m_DepthIntegrated = true;
			if (m_LaserIntegrated && m_DepthIntegrated)
				m_LastLocalizedPose = odomPose.pose;
			m_ReceivedSensorData = true;
		} else {
			m_ParticleFilter->drift(dt);
		}
	} else {
		m_LastLocalizedPose = odomPose.pose;
	}

	m_MotionModel->setLastOdomPose(odomPose);
	m_FirstRun = false;
	m_LastDepthCloudTime = msg->header.stamp;

	publishPoses(m_LastDepthCloudTime);
}

void AMCLDepth::prepareDepthPointCloud(
		sensor_msgs::PointCloud2::ConstPtr const &msg, PointCloud &pc,
		std::vector<float> &ranges) const {

	pcl_conversions::toPCL(msg->header, pc.header);

	// Convert Sensor::PointCloud2 to pcl::PointCloud<Point T>
	PointCloud::Ptr pcTemp(new PointCloud());
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*msg, pcl_pc2);
	pcl::fromPCLPointCloud2(pcl_pc2, *pcTemp);

//	std::cerr << "Sensor to pcl::PointCloud<Point T>" << std::endl;
//	std::cerr << *pcTemp << std::endl;

	// Pass Through filter filtering ranges between min and max ranges
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(pcTemp);
	pass.setFilterFieldName("z");
	// TODO laserMinRange Parameter
	// TODO laserMaxRange Parameter
	double m_FilterMinRange = 1;
	double m_FilterMaxRange = 8;
	pass.setFilterLimits(m_FilterMinRange, m_FilterMaxRange);
	pass.filter(*pcTemp);

//	std::cerr << "Filtered Pass-through pointcloud" << std::endl;
//	std::cerr << *pcTemp << std::endl;

//	// Identify ground plane
//	ROS_INFO("Starting Ground segmentation");
//	PointCloud::Ptr cloudPtr;
//	cloudPtr.reset(new PointCloud(pc));

	pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::SACSegmentation<PointT> seg;
//	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
//	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(pcTemp);
	seg.segment(*inliers, *coef);
	if (inliers->indices.size() == 0) {
		ROS_WARN("couldnt estimate a planar model");
//		return;
	}

	pcl::ExtractIndices<PointT> extract;

	// Extract the outliers(non ground)
	extract.setInputCloud(pcTemp);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*pcTemp);

	// Down sample using leaf size of m
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud(pcTemp);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(pc);

//  std::cerr << "Voxel grid downsampled pointcloud" << std::endl;
//	std::cerr << pc << std::endl;

//	pcl::copyPointCloud(*cloudPtr,inliers->indices,pc);
	// adjust "ranges" array to contain the same points:
	ranges.resize(pc.points.size());
	for (unsigned int i = 0; i < pc.points.size(); ++i) {
		PointT p = pc[i];
		ranges[i] = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
	}
//	std::cerr << "Filtered Segment(non ground) pointcloud" << std::endl;
//	std::cerr << pc << std::endl;

//	ROS_INFO("Filtering DepthCloud Done");
	m_DepthFilteredPub.publish(pc);
}

bool AMCLDepth::globalLocalizationCallback(std_srvs::Empty::Request &req,
		std_srvs::Empty::Response &res) {
	ROS_INFO("Initialize Global Localization with Uniform Distribution");
	double maxX, maxY, maxZ, minX, minY, minZ;
	double minYaw, maxYaw;

	m_MapModel->getMap()->getMetricMax(maxX, maxY, maxZ);
	m_MapModel->getMap()->getMetricMin(minX, minY, minZ);

	minYaw = -M_PI;
	maxYaw = M_PI;

	RobotStateDistribution distribution(minX, maxX, minY, maxY, minYaw, maxYaw);
	m_ParticleFilter->drawAllFromDistribution(distribution);
	m_ParticleFilter->resetTimer();
	m_MotionModel->reset();

	// Do not integrate measurements until moved(??)
	m_ReceivedSensorData = true;
	m_Initialized = true;
	m_FirstRun = true;
	publishPoses(ros::Time::now());

	return true;
}

void AMCLDepth::rgbCallback(sensor_msgs::PointCloud2ConstPtr const &msg) {
	ros::Time tArrived = msg->header.stamp;
	ROS_DEBUG("Received RGB Point Messages at Time: %f",
			msg->header.stamp.toSec());
	if (!m_Initialized) {
		ROS_WARN(
				"Localization not initialized yet, skipping DepthPointCloud input"
						"\n (1) Use RVIZ 2D Pose Estimation button or"
						"\n (2) Publish to /initialpose topic or"
						"\n (3) Start Global Localization Service");
		return;
	}

	double timediff = (tArrived - m_LastDepthCloudTime).toSec();
	if (m_ReceivedSensorData && timediff < 0) {
		ROS_WARN(
				"Ignoring received laser data that is %f s older than previous data!",
				timediff);
		return;
	}

	geometry_msgs::PoseStamped odomPose;
	// Check if Odometry is available otherwise skip scan
	if (!m_MotionModel->lookupOdomPose(tArrived, odomPose)) {
		ROS_WARN("EXITING RGB CLOUD CALLBACK");
		return;
	}

	if (!m_FirstRun) {
		double dt = (odomPose.header.stamp
				- m_MotionModel->getLastOdomPose().header.stamp).toSec();
		m_RGBIntegrated = false;
		if (!m_ReceivedSensorData || isAboveMotionThreshold(odomPose)) {
			PointCloudRGB pcFiltered;
			std::vector<float> rgbRanges;
			prepareRGBPointCloud(msg, pcFiltered, rgbRanges);
			ros::Time t = msg->header.stamp;
			geometry_msgs::TransformStamped sensorToBase;
			if (!m_MotionModel->lookupTargetToBaseTransform(
					pcFiltered.header.frame_id, t, sensorToBase)) {
				return;
			}
			tf2::Transform baseToSensor;
			tf2::convert(sensorToBase.transform, baseToSensor);
			baseToSensor = baseToSensor.inverse();

			// TODO Publish filtered PointCloud

			m_ObservationModel->setRGB(true);
			m_ObservationModel->setBaseToSensorTransform(baseToSensor);
			m_ObservationModel->setObservedMeasurements(pcFiltered, rgbRanges);

			ros::Time past = ros::Time::now();
//			ROS_INFO("RGB filter start in %f s", past.toSec());
			m_ParticleFilter->filter(dt);
			double tdiff = (ros::Time::now() - past).toSec();
			ROS_INFO("RGB filter done in %f s", tdiff);

			m_RGBIntegrated = true;
			if (m_LaserIntegrated && m_RGBIntegrated)
				m_LastLocalizedPose = odomPose.pose;
			m_ReceivedSensorData = true;
		} else {
			m_ParticleFilter->drift(dt);
		}
	} else {
		m_LastLocalizedPose = odomPose.pose;
	}

	m_MotionModel->setLastOdomPose(odomPose);
	m_FirstRun = false;
	m_LastDepthCloudTime = tArrived;

	publishPoses(m_LastDepthCloudTime);
}

void AMCLDepth::prepareRGBPointCloud(
		sensor_msgs::PointCloud2ConstPtr const &msg, PointCloudRGB &pc,
		std::vector<float> &ranges) const {
	pcl_conversions::toPCL(msg->header, pc.header);

//	std::cerr << "Original PoinctCloud" << std::endl;
//	std::cerr << *msg << std::endl;

	// Convert Sensor::PointCloud2 to pcl::PointCloud<PointRGB T>
	PointCloudRGB::Ptr pcTemp(new PointCloudRGB());
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*msg, pcl_pc2);
	pcl::fromPCLPointCloud2(pcl_pc2, *pcTemp);

//	std::cerr << "Sensor to pcl::PointCloud<Point T>" << std::endl;
//	std::cerr << *pcTemp << std::endl;

	// Pass Through filter filtering ranges between min and max ranges
	pcl::PassThrough<PointRGBT> pass;
	pass.setInputCloud(pcTemp);
	pass.setFilterFieldName("z");
	// TODO laserMinRange Parameter
	// TODO laserMaxRange Parameter
	double m_FilterMinRange = 0.5;
	double m_FilterMaxRange = 8;
	pass.setFilterLimits(m_FilterMinRange, m_FilterMaxRange);
	pass.filter(*pcTemp);

//	std::cerr << "Filtered Pass-through pointcloud" << std::endl;
//	std::cerr << *pcTemp << std::endl;

	//	// Identify ground plane
	//	ROS_INFO("Starting Ground segmentation");
	//	PointCloud::Ptr cloudPtr;
	//	cloudPtr.reset(new PointCloud(pc));

	pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::SACSegmentation<PointRGBT> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setAxis(Eigen::Vector3f(0,0,1));
	//	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(pcTemp);
	seg.segment(*inliers, *coef);
	if (inliers->indices.size() == 0) {
		ROS_WARN("couldnt estimate a planar model");
		//		return;
	} else {
		pcl::ExtractIndices<PointRGBT> extract;

		// Extract the outliers(non ground)
		extract.setInputCloud(pcTemp);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*pcTemp);
	}

//	std::cerr << "Filtered Segment(non ground) pointcloud" << std::endl;
//	std::cerr << *pcTemp << std::endl;

	// Down sample using leaf size of m
	pcl::VoxelGrid<PointRGBT> sor;
	sor.setInputCloud(pcTemp);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(pc);

//	std::cerr << "Voxel grid downsampled pointcloud" << std::endl;
//	std::cerr << pc << std::endl;

	//	pcl::copyPointCloud(*cloudPtr,inliers->indices,pc);
	// adjust "ranges" array to contain the same points:
	ranges.resize(pc.points.size());
	for (unsigned int i = 0; i < pc.points.size(); ++i) {
		PointRGBT p = pc[i];
		ranges[i] = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
	}
//	std::cerr << "Filtered Segment(non ground) pointcloud" << std::endl;
//	std::cerr << pc << std::endl;

	//	ROS_INFO("Filtering DepthCloud Done");
	m_DepthFilteredPub.publish(pc);
}

bool AMCLDepth::isAboveMotionThreshold(
		const geometry_msgs::PoseStamped &odomPose) const {
	tf2::Transform lastLocalized;
	tf2::convert(m_LastLocalizedPose, lastLocalized);

	tf2::Transform oPose;
	tf2::convert(odomPose.pose, oPose);

	tf2::Transform odomTransform = lastLocalized.inverseTimes(oPose);

	double yaw, pitch, roll;
	odomTransform.getBasis().getRPY(roll, pitch, yaw);

	//TODO ADD PARAMETERS BELOW
	double m_ObservationThresholdTrans = 0.2;
	double m_ObservationThresholdRot = M_PI / 20;
	bool isAbove = odomTransform.getOrigin().length()
			>= m_ObservationThresholdTrans
			|| std::abs(yaw) >= m_ObservationThresholdRot;
	return isAbove;
}
