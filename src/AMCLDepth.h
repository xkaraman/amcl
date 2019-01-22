/*
 * AMCLDepth.h
 *
 *  Created on: Sep 27, 2018
 *      Author: xenakis
 */

#ifndef SRC_AMCLDEPTH_H_
#define SRC_AMCLDEPTH_H_

#include <amcl_depth_types.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>

#include <libPF/ParticleFilter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <ros/timer.h>

#include <RobotState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <memory>
#include <string>
#include <vector>

class OccupancyMap;
class RobotMovementModel;
class RobotObservationModel;
class RGBObservationModel;

namespace tf2_ros {
class TransformListener;
class TransformBroadcaster;
} /* namespace tf2_ros */

class MapModel;

class AMCLDepth {
public:
	AMCLDepth();

	virtual ~AMCLDepth();

	void publishPoses(const ros::Time &t);

	void computeBestTransform(const ros::Time &t);

	bool globalLocalizationCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

	void initPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

	void laserCallback(const sensor_msgs::LaserScanConstPtr &msg);

	void depthCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

	void rgbCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

	void laserRGBCallback(const sensor_msgs::LaserScanConstPtr &laser,const sensor_msgs::PointCloud2ConstPtr &rgb);

	void prepareLaserPointCloud(const sensor_msgs::LaserScanConstPtr &laser,PointCloud &pc,std::vector<float> &ranges) const;

	void prepareDepthPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg, PointCloud &pc, std::vector<float> &ranges) const;

	void prepareRGBPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg, PointCloudRGB &pc, std::vector<float> &ranges) const;

	void latestTimerCallback(const ros::TimerEvent &e) const;

	bool isAboveMotionThreshold(const geometry_msgs::PoseStamped &odomPose) const;



private:

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,sensor_msgs::PointCloud2> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> *sync;

	std::shared_ptr<MapModel> m_MapModel;
	PointCloudRGB m_MapPointCloud;
	PointCloudRGB::Ptr m_MapDownPointCloud;
//	std::shared_ptr< libPF::MovementModel<RobotState> > m_MovementModel;
	std::shared_ptr< libPF::ObservationModel<RobotState> > m_ObservationModel;

//	OccupancyMap *m_MapModel;
	RobotMovementModel *m_MotionModel;
	RGBObservationModel *m_RGBObs;
//	RobotObservationModel *m_ObservationModel;

	// FrameIDs
	std::string m_MapFrameID;
	std::string m_OdomFrameID;
	std::string m_BaseFrameID;

	int m_NumberOfParticles;
	libPF::ParticleFilter<RobotState> *m_ParticleFilter;

	ros::NodeHandle m_NH;

	tf2_ros::Buffer m_TFBuffer;
	tf2_ros::TransformListener *m_TFListener;
	tf2_ros::TransformBroadcaster *m_TFBroadcaster;
	tf2::Transform m_LatestTransform;

	geometry_msgs::PoseArray m_PoseArray;
	ros::Publisher m_PoseArrayPub;
	ros::Publisher m_DepthFilteredPub;
	ros::Publisher m_BestPosePub;

	ros::Timer m_LatestTransformTimer;

	ros::ServiceServer m_GlobalLocService;

	message_filters::Subscriber<sensor_msgs::LaserScan> *m_LaserScanSub;
	tf2_ros::MessageFilter<sensor_msgs::LaserScan> *m_LaserScanFilter;

	message_filters::Subscriber<sensor_msgs::PointCloud2> *m_DepthScanSub;
	tf2_ros::MessageFilter<sensor_msgs::PointCloud2> *m_DepthScanFilter;

	message_filters::Subscriber<sensor_msgs::PointCloud2> *m_RGBScanSub;
	tf2_ros::MessageFilter<sensor_msgs::PointCloud2> *m_RGBScanFilter;

	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> *m_InitPoseSub;
	tf2_ros::MessageFilter<geometry_msgs::PoseWithCovarianceStamped> *m_InitPoseFilter;

	ros::Time m_LastLaserTime;
	ros::Time m_LastDepthCloudTime;
	ros::Time m_LastRGBCloudTime;
	bool m_LaserIntegrated;
	bool m_DepthIntegrated;
	bool m_RGBIntegrated;
	bool m_UseLaser;
	bool m_UseDepth;
	bool m_UseRGB;

	bool m_ReceivedSensorData;
	bool m_Initialized;
	bool m_FirstRun;
	geometry_msgs::Pose m_LastLocalizedPose;

	double m_SensorSampleDist;
	double m_ObservationThresholdTrans;
	double m_ObservationThresholdRot;
	double m_FilterMinRange;
	double m_FilterMaxRange;
	double m_TransformTolerance;
	int m_ParticlesWithRgb;
	double m_InitXStdDev;
	double m_InitYStdDev;
	double m_InitYawStdDev;
	double m_DownsampleVoxelSize;
	bool m_PublishUpdated;
	double m_Alpha;
};



#endif /* SRC_AMCLDEPTH_H_ */
