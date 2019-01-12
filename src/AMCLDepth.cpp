/*
 * AMCLDepth.cpp
 *
 *  Created on: Sep 27, 2018
 *      Author: xenakis
 */

#include <AMCLDepth.h>

#include <boost/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/connection.h>
#include <message_filters/simple_filter.h>
#include <MapModel.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>

#include <RobotMovementModel.h>
//#include <RobotObservationModel.h>
#include <RGBObservationModel.h>
#include <LaserObservationModel.h>
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
#include <pcl/common/angles.h>
#include <pcl/common/common.h>

AMCLDepth::AMCLDepth() :
		m_NumberOfParticles(500),
		m_MapFrameID("map"), m_OdomFrameID("odom"), m_BaseFrameID("base_footprint"),
		m_ReceivedSensorData(false), m_FirstRun(true),
		m_TFBuffer(ros::Duration(10), false),
		m_MapDownPointCloud(new PointCloudRGB){

	// Get Server Parameters
	m_NH.param("num_particles",m_NumberOfParticles,500);
	m_NH.param("num_particles_rgb",m_ParticlesWithRgb,25);

	m_NH.param("use_laser",m_UseLaser,true);
	m_NH.param("use_depth",m_UseDepth,true);
	m_NH.param("use_rgb",m_UseRGB,true);

	m_NH.param("sensor_sample_distance",m_SensorSampleDist, 0.2);
	m_NH.param("threshold_translation",m_ObservationThresholdTrans,0.3);
	m_NH.param("threshold_rotation",m_ObservationThresholdRot, 0.4);
	m_NH.param("downsample_voxel_size",m_DownsampleVoxelSize,0.2);

	m_NH.param("laser_min_range",m_FilterMinRange, 1.0);
	m_NH.param("laser_max_range",m_FilterMaxRange, 6.0);

	m_NH.param("init/xStdDev",m_InitXStdDev,0.3);
	m_NH.param("init/yStdDev",m_InitYStdDev,0.3);
	m_NH.param("init/yawStdDev",m_InitYawStdDev,0.2);

	m_NH.param("publish_updated",m_PublishUpdated,true);
	m_NH.param("transform_tolerance",m_TransformTolerance, 0.4);

	// Models Used in Particle Filters
	m_MapModel = std::shared_ptr<MapModel>(new OccupancyMap(&m_NH));
//	m_MotionModel = std::shared_ptr<libPF::MovementModel<RobotState> >(new RobotMovementModel(&m_NH,&m_TFBuffer,"odom","map"));
	m_ObservationModel = std::shared_ptr<libPF::ObservationModel<RobotState> >(new LaserObservationModel(&m_NH,m_MapModel));
//	m_MapModel = new OccupancyMap(&m_NH);

	m_RGBObs = new RGBObservationModel(&m_NH,m_MapModel);

	m_MotionModel = new RobotMovementModel(&m_NH, &m_TFBuffer, m_OdomFrameID,
			m_BaseFrameID);
//	m_ObservationModel = new RobotObservationModel(&m_NH, m_MapModel);

	m_ParticleFilter = new libPF::ParticleFilter<RobotState>(
			m_NumberOfParticles, m_ObservationModel.get(), m_MotionModel);

	m_MapPointCloud = ((OccupancyMap*)(m_MapModel.get()))->toPCL();
	pcl::VoxelGrid<PointRGBT> vg;
	vg.setInputCloud(m_MapPointCloud.makeShared());
	vg.setLeafSize(m_DownsampleVoxelSize,m_DownsampleVoxelSize,m_DownsampleVoxelSize);
	vg.filter(*m_MapDownPointCloud);
	m_RGBObs->setScenePointCloud(m_MapDownPointCloud);
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
	m_BestPosePub = m_NH.advertise<geometry_msgs::PoseStamped>("amcl_depth/bestpose",10);
	m_PoseArrayPub = m_NH.advertise<geometry_msgs::PoseArray>(
			"amcl_depth/particlecloud", 10);
	m_DepthFilteredPub = m_NH.advertise<PointCloud>("amcl_depth/depthfiltered",
			20);

	// Advertise Services and Initialize Subscribers
	m_GlobalLocService = m_NH.advertiseService("amcl_depth/global_localization",
			&AMCLDepth::globalLocalizationCallback, this);

	m_LaserScanSub = new message_filters::Subscriber<sensor_msgs::LaserScan>(
			m_NH, "/scan", 100);

	m_DepthScanSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(
			m_NH, "/camera/depth/points", 100);

	m_RGBScanSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(m_NH,
			"/camera/depth/points", 100);

	if( m_UseLaser && !(m_UseDepth || m_UseRGB)){
		m_LaserScanFilter = new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(
			*m_LaserScanSub, m_TFBuffer, m_OdomFrameID, 100, m_NH);
		if(m_UseLaser)
		m_LaserScanFilter->registerCallback(boost::bind(&AMCLDepth::laserCallback,this,_1));
	} else {
		sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),*m_LaserScanSub,*m_RGBScanSub);
		sync->registerCallback(boost::bind(&AMCLDepth::laserRGBCallback,this,_1,_2));
	}

//	m_DepthScanFilter = new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(
//			*m_DepthScanSub, m_TFBuffer, m_OdomFrameID, 100, m_NH);
//	if(m_UseDepth)
//		m_DepthScanFilter->registerCallback(boost::bind(&AMCLDepth::depthCallback,this,_1));


//	m_RGBScanFilter = new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*m_RGBScanSub,
//			m_TFBuffer, m_OdomFrameID, 100, m_NH);
//	if(m_UseRGB)
//		m_RGBScanFilter->registerCallback(boost::bind(&AMCLDepth::rgbCallback, this, _1));

	m_InitPoseSub = new message_filters::Subscriber<
			geometry_msgs::PoseWithCovarianceStamped>(m_NH, "initialpose", 2);
	m_InitPoseFilter = new tf2_ros::MessageFilter<
			geometry_msgs::PoseWithCovarianceStamped>(*m_InitPoseSub,
			m_TFBuffer, m_MapFrameID, 2, m_NH);
	m_InitPoseFilter->registerCallback(
			boost::bind(&AMCLDepth::initPoseCallback, this, _1));

	// Timer for resending the latest transform
	m_LatestTransformTimer = m_NH.createTimer(ros::Duration(m_TransformTolerance),
			&AMCLDepth::latestTimerCallback, this);
//	ROS_INFO("Using timer with a period of %4f s", mTimerPeriod);

	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
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
				"LaserScan::Ignoring received laser data that is %f s older than previous data!",
				timediff);
		return;
	}

	geometry_msgs::PoseStamped odomPose;
	// Check if Odometry is available otherwise skip scan
	if (!m_MotionModel->lookupOdomPose(msg->header.stamp, odomPose)) {
		ROS_WARN("LaserScan::No Odometry Pose found");
		return;
	}

	if (!m_FirstRun) {
		ros::Time start = ros::Time::now();
		double dt = (odomPose.header.stamp
				- m_MotionModel->getLastOdomPose().header.stamp).toSec();
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

//			m_ObservationModel = std::shared_ptr<libPF::ObservationModel<RobotState> >( new LaserObservationModel(&m_NH,m_MapModel));
			LaserObservationModel* laser = (LaserObservationModel*)m_ObservationModel.get();
			laser->setBaseToSensorTransform(baseToSensor);
			laser->setObservedMeasurements(pcFiltered,laserRanges);
//			m_ObservationModel->setBaseToSensorTransform(baseToSensor);
//			m_ObservationModel->setObservedMeasurements(pcFiltered,
//					laserRanges);

			m_ParticleFilter->setObservationModel(laser);

			ros::Time past = ros::Time::now();
//			ROS_INFO("Laser filter start in %f s", past.toSec());
			m_ParticleFilter->filter(dt);
			double tdiff = (ros::Time::now() - start).toSec();
			ROS_INFO("Laser filter done in %f s", tdiff);

			if(m_PublishUpdated)
				publishPoses(msg->header.stamp);
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
	if(!m_PublishUpdated)
		publishPoses(m_LastLaserTime);

}

void AMCLDepth::initPoseCallback(
		geometry_msgs::PoseWithCovarianceStampedConstPtr const &msg) {

	tf2::Transform pose;
	tf2::convert(msg->pose.pose, pose);

	ROS_INFO("Setting pose near (%.6f): x=%.3f y= %.3f yaw=%.3f",
			ros::Time::now().toSec(), pose.getOrigin().getX(),
			pose.getOrigin().getY(), tf2::getYaw(pose.getRotation()));

	RobotStateDistribution distribution(pose.getOrigin().getX(),pose.getOrigin().getY(),tf2::getYaw(pose.getRotation()));
	distribution.setUniform(false);
	distribution.setStdDev(m_InitXStdDev,m_InitYStdDev,m_InitYawStdDev);
	m_ParticleFilter->drawAllFromDistribution(distribution);
	m_ParticleFilter->setResamplingMode(libPF::RESAMPLE_NEFF);
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
	transform.header.stamp = e.current_real + ros::Duration(m_TransformTolerance);//  ros::Time::now() + ros::Duration(0.1);
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

			pc.points.push_back(PointT(pt.x(), pt.y(), pt.z()));
			ranges.push_back(range);
		} else {
			numBeamsSkipped++;
		}
	}

	pc.is_dense = true;

	// Uniform Sampling
	pcl::UniformSampling<PointT> uniformSampling;
	PointCloud::Ptr cloudPtr;
	cloudPtr.reset(new PointCloud(pc));
	uniformSampling.setInputCloud(cloudPtr);

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
//	ROS_INFO("Laser PointCloud subsampled: %zu from %zu (%u out of valid range)",
//			pc.size(), cloudPtr->size(), numBeamsSkipped);
}

void AMCLDepth::publishPoses(const ros::Time &t) {
	m_PoseArray.header.stamp = t;
	if (m_PoseArray.poses.size() != m_ParticleFilter->numParticles()) {
		m_PoseArray.poses.resize(m_ParticleFilter->numParticles());
	}

	for (unsigned i = 0; i < m_ParticleFilter->numParticles(); i++) {
		m_PoseArray.poses[i] = m_ParticleFilter->getState(i).getPose();
	}

	geometry_msgs::PoseStamped bestPose;
	bestPose.header.stamp = t;
	bestPose.header.frame_id = m_MapFrameID;
	bestPose.pose = m_ParticleFilter->getState(0).getPose();
	m_BestPosePub.publish(bestPose);
	m_PoseArrayPub.publish(m_PoseArray);

	computeBestTransform(t);

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

//void AMCLDepth::depthCallback(sensor_msgs::PointCloud2ConstPtr const &msg) {
//	ROS_DEBUG("Received Depth Messages at Time: %f", msg->header.stamp.toSec());
//	if (!m_Initialized) {
//		ROS_WARN(
//				"Localization not initialized yet, skipping DepthPointCloud input"
//						"\n (1) Use RVIZ 2D Pose Estimation button or"
//						"\n (2) Publish to /initialpose topic or"
//						"\n (3) Start Global Localization Service");
//		return;
//	}
//
//	double timediff = (msg->header.stamp - m_LastDepthCloudTime).toSec();
//	if (m_ReceivedSensorData && timediff < 0) {
//		ROS_WARN(
//				"Ignoring received laser data that is %f s older than previous data!",
//				timediff);
//		return;
//	}
//
//	geometry_msgs::PoseStamped odomPose;
//	// Check if Odometry is available otherwise skip scan
//	if (!m_MotionModel->lookupOdomPose(msg->header.stamp, odomPose)) {
//		ROS_WARN("EXITING DEPTH CLOUD CALLBACK");
//		return;
//	}
//
//	if (!m_FirstRun) {
//		double dt = (odomPose.header.stamp
//				- m_MotionModel->getLastOdomPose().header.stamp).toSec();
//		m_DepthIntegrated = false;
//		if (!m_ReceivedSensorData || isAboveMotionThreshold(odomPose)) {
//			PointCloud pcFiltered;
//			std::vector<float> laserRanges;
//			prepareDepthPointCloud(msg, pcFiltered, laserRanges);
//			ros::Time t = msg->header.stamp;
//			geometry_msgs::TransformStamped sensorToBase;
//			if (!m_MotionModel->lookupTargetToBaseTransform(
//					pcFiltered.header.frame_id, t, sensorToBase)) {
//				return;
//			}
//			tf2::Transform baseToSensor;
//			tf2::convert(sensorToBase.transform, baseToSensor);
//			baseToSensor = baseToSensor.inverse();
//
//			// TODO Publish filtered PointCloud
//
//			m_ObservationModel = std::shared_ptr<libPF::ObservationModel<RobotState> >( new RobotObservationModel(&m_NH, m_MapModel));
//			RobotObservationModel* depth;
//			depth = (RobotObservationModel*)m_ObservationModel.get();
//			depth->setRGB(false);
//			depth->setBaseToSensorTransform(baseToSensor);
//			depth->setObservedMeasurements(pcFiltered,laserRanges);
//
//			m_ParticleFilter->setObservationModel(depth);
////			m_ObservationModel->setRGB(false);
////			m_ObservationModel->setBaseToSensorTransform(baseToSensor);
////			m_ObservationModel->setObservedMeasurements(pcFiltered,
////					laserRanges);
//
//			ros::Time past = ros::Time::now();
////			ROS_INFO("Depth filter start in %f s", past.toSec());
//			m_ParticleFilter->filter(dt);
//			double tdiff = (ros::Time::now() - past).toSec();
//			ROS_INFO("Depth filter done in %f s", tdiff);
//
//			m_DepthIntegrated = true;
//			if(!m_UseLaser) m_LaserIntegrated = true;
//			if(!m_UseRGB) m_RGBIntegrated = true;
//
//			if (m_LaserIntegrated && m_DepthIntegrated && m_RGBIntegrated)
//				m_LastLocalizedPose = odomPose.pose;
//			m_ReceivedSensorData = true;
//		} else {
//			m_ParticleFilter->drift(dt);
//		}
//	} else {
//		m_LastLocalizedPose = odomPose.pose;
//	}
//
//	m_MotionModel->setLastOdomPose(odomPose);
//	m_FirstRun = false;
//	m_LastDepthCloudTime = msg->header.stamp;
//
//	publishPoses(m_LastDepthCloudTime);
//}

//void AMCLDepth::prepareDepthPointCloud(
//		sensor_msgs::PointCloud2::ConstPtr const &msg, PointCloud &pc,
//		std::vector<float> &ranges) const {
//
//	pcl_conversions::toPCL(msg->header, pc.header);
//
//	// Convert Sensor::PointCloud2 to pcl::PointCloud<Point T>
//	PointCloud::Ptr pcTemp(new PointCloud());
//	pcl::PCLPointCloud2 pcl_pc2;
//	pcl_conversions::toPCL(*msg, pcl_pc2);
//	pcl::fromPCLPointCloud2(pcl_pc2, *pcTemp);
//
////	std::cerr << "Sensor to pcl::PointCloud<Point T>" << std::endl;
////	std::cerr << *pcTemp << std::endl;
//
//	// Pass Through filter filtering ranges between min and max ranges
//	pcl::PassThrough<PointT> pass;
//	pass.setInputCloud(pcTemp);
//	pass.setFilterFieldName("z");
//	pass.setFilterLimits(m_FilterMinRange, m_FilterMaxRange);
//	pass.filter(*pcTemp);
//
////	std::cerr << "Filtered Pass-through pointcloud" << std::endl;
////	std::cerr << *pcTemp << std::endl;
//
////	// Identify ground plane
////	ROS_INFO("Starting Ground segmentation");
////	PointCloud::Ptr cloudPtr;
////	cloudPtr.reset(new PointCloud(pc));
//
//	pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients());
//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//	pcl::SACSegmentation<PointT> seg;
////	seg.setOptimizeCoefficients(true);
//	seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
//	seg.setMethodType(pcl::SAC_RANSAC);
//	seg.setAxis(Eigen::Vector3f(0.0,0.0,1.0));
//	seg.setEpsAngle(pcl::deg2rad(10.0));
////	seg.setMaxIterations(1000);
//	seg.setDistanceThreshold(0.01);
//	seg.setInputCloud(pcTemp);
//	seg.segment(*inliers, *coef);
//	if (inliers->indices.size() == 0) {
//		ROS_WARN("couldnt estimate a planar model");
////		return;
//	}
//
//	pcl::ExtractIndices<PointT> extract;
//
//	// Extract the outliers(non ground)
//	extract.setInputCloud(pcTemp);
//	extract.setIndices(inliers);
//	extract.setNegative(true);
//	extract.filter(*pcTemp);
//
//	// Down sample using leaf size of m
//	pcl::VoxelGrid<PointT> sor;
//	sor.setInputCloud(pcTemp);
//	sor.setLeafSize(0.5f, 0.5f, 0.5f);
//	sor.filter(pc);
//
////  std::cerr << "Voxel grid downsampled pointcloud" << std::endl;
////	std::cerr << pc << std::endl;
//
////	pcl::copyPointCloud(*cloudPtr,inliers->indices,pc);
//	// adjust "ranges" array to contain the same points:
//	ranges.resize(pc.points.size());
//	for (unsigned int i = 0; i < pc.points.size(); ++i) {
//		PointT p = pc[i];
//		ranges[i] = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
//	}
////	std::cerr << "Filtered Segment(non ground) pointcloud" << std::endl;
////	std::cerr << pc << std::endl;
//
////	ROS_INFO("Filtering DepthCloud Done");
//	m_DepthFilteredPub.publish(pc);
//}

bool AMCLDepth::globalLocalizationCallback(std_srvs::Empty::Request &req,
		std_srvs::Empty::Response &res) {
	ROS_INFO("Initialize Global Localization with Uniform Distribution");
	double maxX, maxY, maxZ, minX, minY, minZ;
	double minYaw, maxYaw;

	m_MapModel->getMap()->getMetricMax(maxX, maxY, maxZ);
	m_MapModel->getMap()->getMetricMin(minX, minY, minZ);

	minYaw = -M_PI;
	maxYaw = M_PI;

//	RobotStateDistribution distribution(minX, maxX, minY, maxY, minYaw, maxYaw);
	RobotStateDistribution distribution(m_MapModel);
	distribution.setUniform(true);
	m_ParticleFilter->drawAllFromDistribution(distribution);
	m_ParticleFilter->setResamplingMode(libPF::RESAMPLE_NEFF);
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
	ros::Time start = ros::Time::now();

	ROS_DEBUG("Received RGB Point Messages at Time: %f",
			msg->header.stamp.toSec());
	if (!m_Initialized) {
		ROS_WARN(
				"Localization not initialized yet, skipping RGBPointCloud input"
						"\n (1) Use RVIZ 2D Pose Estimation button or"
						"\n (2) Publish to /initialpose topic or"
						"\n (3) Start Global Localization Service");
		return;
	}

	double timediff = (tArrived - m_LastRGBCloudTime).toSec();
	if (m_ReceivedSensorData && timediff < 0) {
		ROS_WARN(
				"Ignoring received rgb data that is %f s older than previous data!",
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

//			prepareRGBPointCloud(msg, pcFiltered, rgbRanges);
			PointCloudRGB::Ptr pcTemp(new PointCloudRGB());
			pcl::PCLPointCloud2 pcl_pc2;
			pcl_conversions::toPCL(*msg, pcl_pc2);
			pcl::fromPCLPointCloud2(pcl_pc2, *pcTemp);

			ros::Time t = msg->header.stamp;
			geometry_msgs::TransformStamped sensorToBase;
			if (!m_MotionModel->lookupTargetToBaseTransform(
					msg->header.frame_id, t, sensorToBase)) {
				return;
			}
			tf2::Transform baseToSensor;
			tf2::convert(sensorToBase.transform, baseToSensor);
			baseToSensor = baseToSensor.inverse();

			// TODO Publish filtered PointCloud
//			m_ObservationModel = std::shared_ptr<libPF::ObservationModel<RobotState> >( new RGBObservationModel(&m_NH,m_MapModel));
//			RGBObservationModel* rgb;
//			rgb = (RGBObservationModel*)m_ObservationModel.get();
			m_RGBObs->setBaseToSensorTransform(baseToSensor);

			// Down sample using leaf size of m
			pcl::VoxelGrid<PointRGBT> vg;
			vg.setInputCloud(pcTemp);
			vg.setLeafSize(m_DownsampleVoxelSize, m_DownsampleVoxelSize, m_DownsampleVoxelSize);
			vg.setFilterFieldName("z");
			vg.setFilterLimits(m_FilterMinRange,m_FilterMaxRange);
			vg.filter(*pcTemp);
			m_RGBObs->setObservedMeasurements(pcTemp);
			m_RGBObs->setRGB(m_UseRGB);
			m_ParticleFilter->setObservationModel(m_RGBObs);
//			m_ObservationModel->setRGB(true);
//			m_ObservationModel->setBaseToSensorTransform(baseToSensor);
//			m_ObservationModel->setObservedMeasurements(pcFiltered, rgbRanges);

			ros::Time past = ros::Time::now();
//			ROS_INFO("RGB filter start in %f s", past.toSec());
			m_ParticleFilter->filter(dt);
			double tdiff = (ros::Time::now() - start).toSec();
			ROS_INFO("RGB filter done in %f s", tdiff);

			m_RGBIntegrated = true;
			if(!m_UseDepth) m_DepthIntegrated = true;
			if(!m_UseLaser) m_LaserIntegrated = true;

			if(m_PublishUpdated)
				publishPoses(msg->header.stamp);
			if (m_LaserIntegrated && m_DepthIntegrated && m_RGBIntegrated)
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
	m_LastRGBCloudTime = msg->header.stamp;
	if(!m_PublishUpdated)
		publishPoses(m_LastRGBCloudTime);
}
//
//void AMCLDepth::prepareRGBPointCloud(
//		sensor_msgs::PointCloud2ConstPtr const &msg, PointCloudRGB &pc,
//		std::vector<float> &ranges) const {
//	pcl_conversions::toPCL(msg->header, pc.header);
//
////	std::cerr << "Original PoinctCloud" << std::endl;
////	std::cerr << *msg << std::endl;
//
////	pcl::PCDWriter writer;
//	// Convert Sensor::PointCloud2 to pcl::PointCloud<PointRGB T>
//	PointCloudRGB::Ptr pcTemp(new PointCloudRGB());
//	pcl::PCLPointCloud2 pcl_pc2;
//	pcl_conversions::toPCL(*msg, pcl_pc2);
//	pcl::fromPCLPointCloud2(pcl_pc2, *pcTemp);
//
////	pcl::io::savePCDFileASCII<PointRGBT> ("sensor.pcd", *pcTemp);
////	std::cerr << "Saved " << pcTemp->points.size () << " data points to test_pcd.pcd." << std::endl;
////	std::cerr << "Sensor to pcl::PointCloud<Point T>" << std::endl;
////	std::cerr << *pcTemp << std::endl;
//
//	// Pass Through filter filtering ranges between min and max ranges
//	pcl::PassThrough<PointRGBT> pass;
//	pass.setInputCloud(pcTemp);
//	pass.setFilterFieldName("z");
//	pass.setFilterLimits(m_FilterMinRange, m_FilterMaxRange);
//	pass.filter(*pcTemp);
//
////	std::cerr << "Filtered Pass-through pointcloud" << std::endl;
////	std::cerr << *pcTemp << std::endl;
//
//	//	// Identify ground plane
//	//	ROS_INFO("Starting Ground segmentation");
//	//	PointCloud::Ptr cloudPtr;
//	//	cloudPtr.reset(new PointCloud(pc));
//
//	pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients());
//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//	pcl::SACSegmentation<PointRGBT> seg;
//	seg.setOptimizeCoefficients(true);
//	seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
//	seg.setMethodType(pcl::SAC_RANSAC);
//	seg.setAxis(Eigen::Vector3f(0,0,1));
//	//	seg.setMaxIterations(1000);
//	seg.setDistanceThreshold(0.01);
//	seg.setInputCloud(pcTemp);
//	seg.segment(*inliers, *coef);
//	if (inliers->indices.size() == 0) {
//		ROS_WARN("couldnt estimate a planar model");
//		//		return;
//	} else {
//		pcl::ExtractIndices<PointRGBT> extract;
//
//		// Extract the outliers(non ground)
//		extract.setInputCloud(pcTemp);
//		extract.setIndices(inliers);
//		extract.setNegative(true);
//		extract.filter(*pcTemp);
//	}
//
////	std::cerr << "Filtered Segment(non ground) pointcloud" << std::endl;
////	std::cerr << *pcTemp << std::endl;
//
//	// Down sample using leaf size of m
//	pcl::VoxelGrid<PointRGBT> sor;
//	sor.setInputCloud(pcTemp);
//	sor.setLeafSize(0.5f, 0.5f, 0.5f);
//	sor.filter(pc);
//
////	std::cerr << "Voxel grid downsampled pointcloud" << std::endl;
////	std::cerr << pc << std::endl;
//
//	//	pcl::copyPointCloud(*cloudPtr,inliers->indices,pc);
//	// adjust "ranges" array to contain the same points:
//	ranges.resize(pc.points.size());
//	for (unsigned int i = 0; i < pc.points.size(); ++i) {
//		PointRGBT p = pc[i];
//		ranges[i] = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
//	}
////	std::cerr << "Filtered Segment(non ground) pointcloud" << std::endl;
////	std::cerr << pc << std::endl;
//
//	//	ROS_INFO("Filtering DepthCloud Done");
//	m_DepthFilteredPub.publish(pc);
//}

void AMCLDepth::laserRGBCallback(sensor_msgs::LaserScanConstPtr const & laser,
		sensor_msgs::PointCloud2ConstPtr const & rgb) {

	ROS_DEBUG("Received Laser Messages at Time: %f", laser->header.stamp.toSec());
	if (!m_Initialized) {
		ROS_WARN("Localization not initialized yet, skipping LaserScan and RGB input"
				"\n (1) Use RVIZ 2D Pose Estimation button or"
				"\n (2) Publish to /initialpose topic or"
				"\n (3) Start Global Localization Service");
		return;
	}

	double timediff = (laser->header.stamp - m_LastLaserTime).toSec();
	if (m_ReceivedSensorData && timediff < 0) {
		ROS_WARN(
				"Ignoring received data that is %f s older than previous data!",
				timediff);
		return;
	}


	geometry_msgs::PoseStamped odomPose;
	// Check if Odometry is available otherwise skip scan
	if (!m_MotionModel->lookupOdomPose(laser->header.stamp, odomPose)) {
		ROS_WARN("AMCLDepth::laserRGBCallback():Odometry Pose not found");
		return;
	}

	if (!m_FirstRun) {
		ros::Time start = ros::Time::now();
		double dt = (odomPose.header.stamp - m_MotionModel->getLastOdomPose().header.stamp).toSec();
		m_LaserIntegrated = false;
		if (!m_ReceivedSensorData || isAboveMotionThreshold(odomPose)) {
			PointCloud pcFiltered;
			std::vector<float> laserRanges;
			prepareLaserPointCloud(laser, pcFiltered, laserRanges);

			ros::Time t = laser->header.stamp;
			geometry_msgs::TransformStamped sensorToBase;
			if (!m_MotionModel->lookupTargetToBaseTransform(
					pcFiltered.header.frame_id, t, sensorToBase)) {
				return;
			}
			tf2::Transform baseToSensor;
			tf2::convert(sensorToBase.transform, baseToSensor);
			baseToSensor = baseToSensor.inverse();

			// TODO Publish filtered PointCloud

			((LaserObservationModel*)(m_ObservationModel.get()))->setBaseToSensorTransform(baseToSensor);
			((LaserObservationModel*)(m_ObservationModel.get()))->setObservedMeasurements(pcFiltered,laserRanges);

			ros::Time laserTime = ros::Time::now();
//			ROS_INFO("Laser filter start in %f s", past.toSec());
			m_ParticleFilter->filter(dt);
			double tdiff = (ros::Time::now() - laserTime).toSec();
//			ROS_INFO("Laser filter done in %f s", tdiff);

			// Convert to PCL Compatible and Downsample (input data)
			PointCloudRGB::Ptr pcTemp(new PointCloudRGB());
			pcl::PCLPointCloud2 pcl_pc2;
			pcl_conversions::toPCL(*rgb, pcl_pc2);
			pcl::fromPCLPointCloud2(pcl_pc2, *pcTemp);

//			PointRGBT minoriginal,maxoriginal;
//			pcl::getMinMax3D<PointRGBT>(*pcTemp,minoriginal,maxoriginal);

//			std::cout<<"Min Point at: " << minoriginal << std::endl;
//			std::cout<<"Max Point at: " << maxoriginal << std::endl;

//			DownSample Sensor Cloud
			pcl::VoxelGrid<PointRGBT> vg;
			vg.setInputCloud(pcTemp);
			vg.setLeafSize(m_DownsampleVoxelSize, m_DownsampleVoxelSize, m_DownsampleVoxelSize);
			vg.setFilterFieldName("z");
			vg.setFilterLimits(m_FilterMinRange,m_FilterMaxRange);
			vg.filter(*pcTemp);

//			pcl::getMinMax3D<PointRGBT>(*pcTemp,minoriginal,maxoriginal);

//			std::cout<<"Min Point Downsampled at: " << minoriginal << std::endl;
//			std::cout<<"Max Point Downsampled at: " << maxoriginal << std::endl;

			t = rgb->header.stamp;
			if (!m_MotionModel->lookupTargetToBaseTransform(
					rgb->header.frame_id, t, sensorToBase)) {
				return;
			}
			tf2::convert(sensorToBase.transform, baseToSensor);
			baseToSensor = baseToSensor.inverse();

			m_RGBObs->setBaseToSensorTransform(baseToSensor);
			m_RGBObs->setObservedMeasurements(pcTemp);
//			m_RGBObs->setMinMax3D(min,max);

			if(m_UseRGB)
				m_RGBObs->setRGB(true);
			else
				m_RGBObs->setRGB(false);

			ros::Time rgbTime = ros::Time::now();
			unsigned rgbPartToUse = std::min(m_ParticlesWithRgb,m_NumberOfParticles);
			std::vector<int> shuffled(m_NumberOfParticles);
			std::iota(shuffled.begin(),shuffled.end(),0);
			std::shuffle(shuffled.begin(),shuffled.end(),std::default_random_engine());

			std::vector<double> weights;
			weights.resize(rgbPartToUse);
			#pragma omp parallel for shared(shuffled,weights)
			for(int it = 0; it < rgbPartToUse; it++){
				RobotState temp;
				double weight;
				int pick = shuffled[it];
				temp = m_ParticleFilter->getParticle(pick)->getState();

//				weight = m_ParticleFilter->getParticle(pick)->getWeight();
//				weight = weight * m_RGBObs->measure(temp);
//				m_ParticleFilter->getParticle(pick)->setWeight(weight);

				//// Coherence ////////
				weights[it] = (m_RGBObs->measure(temp));
			}
			// Normalize Coherence weights
			auto result = std::minmax_element(weights.begin(),weights.end());
			double min = weights[result.first - weights.begin()];
			double max = weights[result.second - weights.begin()];
			double sum = 0.0;
			for (int it = 0; it < rgbPartToUse; it++){
//				std::cout << "weight before:" << weights[it] << ' ' << min << ' ' << max << std::endl;
				weights[it] = std::exp(1.0 - 15.0*(weights[it]-min)/(max-min));
//				std::cout << "weight after:" << weights[it]<< std::endl;
				sum += weights[it];
			}

			for (int it = 0; it < rgbPartToUse; it++){
				weights[it] = weights[it] / sum ;
			}

			for (int it = 0; it < rgbPartToUse; it++){
				int pick = shuffled[it];
				double weight;
				weight = m_ParticleFilter->getParticle(pick)->getWeight();
				weight = weight * weights[it];
				m_ParticleFilter->getParticle(pick)->setWeight(weight);
			}

			///////////////////////////
			m_ParticleFilter->sort();
			m_ParticleFilter->normalize();
			tdiff = (ros::Time::now() - rgbTime).toSec();
//			ROS_INFO("RGB filter done in %f s", tdiff);

			if(m_PublishUpdated)
				publishPoses(rgb->header.stamp);
			m_LastLocalizedPose = odomPose.pose;
			m_ReceivedSensorData = true;

			tdiff = (ros::Time::now() - start).toSec();
			ROS_INFO("Filter done in %f s", tdiff);
		} else {
			m_ParticleFilter->drift(dt);
//			ROS_INFO("laser drift");
		}
	} else {
		m_LastLocalizedPose = odomPose.pose;
//		ROS_INFO("Saved Pose...first run");
	}

	m_MotionModel->setLastOdomPose(odomPose);
	m_FirstRun = false;
	m_LastLaserTime = laser->header.stamp;
	m_LastRGBCloudTime = rgb->header.stamp;

//	ros::Time publishTime = ros::Time::now();
	if(!m_PublishUpdated)
		publishPoses(m_LastRGBCloudTime);
//	double tdiff = (ros::Time::now() - publishTime).toSec();
//	ROS_INFO("Publish done in %f s", tdiff);
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

	bool isAbove = odomTransform.getOrigin().length()
			>= m_ObservationThresholdTrans
			|| std::abs(yaw) >= m_ObservationThresholdRot;
	return isAbove;
}
