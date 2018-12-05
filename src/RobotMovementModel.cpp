#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <libPF/CRandomNumberGenerator.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>
#include <RobotMovementModel.h>
#include <std_msgs/Header.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>

//#include <string>
namespace ros {
class NodeHandle;
} /* namespace ros */

/* This function is only useful to have the whole code work
 * with old rosbags that have trailing slashes for their frames
 * switch to tf2 by vrabaud slam_gmapping
 */
std::string stripSlash(const std::string &in)
{
  std::string out = in;
  if ( ( !in.empty() ) && (in[0] == '/') )
    out.erase(0,1);
  return out;
}
RobotMovementModel::RobotMovementModel(ros::NodeHandle *nh,
		tf2_ros::Buffer *tfBuffer, const std::string &odomFrameID,
		const std::string &baseFrameID) :
		libPF::MovementModel<RobotState>(),
		m_TFBuffer(tfBuffer),
		m_TFListener(new tf2_ros::TransformListener(*m_TFBuffer)),
		m_OdomFrameID(odomFrameID), m_BaseFrameID(baseFrameID),
		m_FirstOdometryReceived(false)
	{
	m_RNG = new libPF::CRandomNumberGenerator();
	nh->param("motion/xStdDev",m_XStdDev,0.05);
	nh->param("motion/yStdDev",m_YStdDev,0.05);
	nh->param("motion/yawStdDev",m_ThetaStdDev,0.1);
	ROS_INFO("Robot Motion Model Created!");
}

RobotMovementModel::~RobotMovementModel() {
	delete m_RNG;
}

void RobotMovementModel::drift(RobotState& state, double dt) const {
	geometry_msgs::Pose pose;
	pose = state.getPose();
//	ROS_INFO("Pose x=[%f] y=[%f] yaw=[%f]",pose.position.x,pose.position.y,tf2::getYaw(pose.orientation));

	geometry_msgs::TransformStamped odomTransform;
	if (!lookupOdomTransform(m_LastOdomPose.header.stamp + ros::Duration(dt), odomTransform)) {
		ROS_WARN("MovementModel::Drift(): Transform not found");
		return;
	}

	applyOdomTransform(pose,odomTransform);
//	ROS_INFO("Pose after Transform x=[%f] y=[%f] yaw=[%f]",pose.position.x,pose.position.y,tf2::getYaw(pose.orientation));

	state.setPose(pose);
}

void RobotMovementModel::diffuse(RobotState& state, double dt) const {
    state.setXPos(state.getXPos() + m_RNG->getGaussian(m_XStdDev) * dt );
    state.setYPos(state.getYPos() + m_RNG->getGaussian(m_YStdDev) * dt );
    state.setTheta(state.getTheta() + m_RNG->getGaussian(m_ThetaStdDev) * dt  );
}

void RobotMovementModel::setXStdDev(double d) {
	m_XStdDev = d;
}

double RobotMovementModel::getXStdDev() const {
	return m_XStdDev;
}

void RobotMovementModel::setYStdDev(double d) {
	m_YStdDev = d;
}

double RobotMovementModel::getYStdDev() const {
	return m_YStdDev;
}

void RobotMovementModel::setThetaStdDev(double d) {
	m_ThetaStdDev = d;
}

double RobotMovementModel::getThetaStdDev() const {
	return m_ThetaStdDev;
}

bool RobotMovementModel::lookupOdomPose(ros::Time const &t,
		geometry_msgs::PoseStamped &odomPose) const {
	geometry_msgs::PoseStamped identity;
	identity.header.frame_id = m_BaseFrameID;
	identity.header.stamp = t;

	tf2::toMsg(tf2::Transform::getIdentity(), identity.pose);
	try {
		m_TFBuffer->transform(identity, odomPose, m_OdomFrameID,ros::Duration(0.1));
	} catch (tf2::TransformException& e) {
		ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
		return false;
	}

	return true;
}

bool RobotMovementModel::lookupOdomTransform(ros::Time const &t,
		geometry_msgs::TransformStamped &odomTransform) const {
	geometry_msgs::PoseStamped odomPose;

	if (t < m_LastOdomPose.header.stamp) {
		ROS_WARN(
				"Looking up OdomTransform that is %f ms older than the last odomPose!",
				(m_LastOdomPose.header.stamp - t).toSec() / 1000.0);
	}

	if (!lookupOdomPose(t, odomPose)) {
		return false;
	}

	odomTransform = computeOdomTransform(odomPose);
	return true;

}

geometry_msgs::TransformStamped RobotMovementModel::computeOdomTransform(
		geometry_msgs::PoseStamped const &currentPose) const {

	tf2::Transform lastOdomPose;
	tf2::fromMsg(m_LastOdomPose.pose, lastOdomPose);

	tf2::Transform curPose;
	tf2::fromMsg(currentPose.pose, curPose);

	geometry_msgs::TransformStamped transform;

	if (m_FirstOdometryReceived) {
		tf2::convert(lastOdomPose.inverseTimes(curPose), transform.transform);
		return transform;
	} else {
		tf2::convert( tf2::Transform::getIdentity(), transform.transform);
		return transform;
	}
}

void RobotMovementModel::applyOdomTransform(geometry_msgs::Pose &statePose,
		geometry_msgs::TransformStamped const &odomTransform) const {

	tf2::Transform poseTF;
	tf2::fromMsg(statePose,poseTF);

	tf2::Transform odomTransformTF;
	tf2::fromMsg(odomTransform.transform,odomTransformTF);

	tf2::Transform result;
	result = poseTF * odomTransformTF;

	tf2::toMsg(result,statePose);

// 	Not the same with TF * TF ( vector addition?)
//	tf2::doTransform(statePose,statePose,odomTransform);
}

void RobotMovementModel::setLastOdomPose(geometry_msgs::PoseStamped &pose) {
	m_FirstOdometryReceived = true;
	if(pose.header.stamp < m_LastOdomPose.header.stamp){
	    ROS_WARN("Trying to store an OdomPose that is older than the current in the MotionModel, ignoring!");
	} else {
	m_LastOdomPose = pose;
	}
}
geometry_msgs::PoseStamped RobotMovementModel::getLastOdomPose() const {
	return m_LastOdomPose;
}

void RobotMovementModel::reset(){
	m_FirstOdometryReceived = false;
}

bool RobotMovementModel::lookupTargetToBaseTransform(
		std::string const &targetFrame, ros::Time const &t,
		geometry_msgs::TransformStamped &localTransform) const {
	try {
		localTransform = m_TFBuffer->lookupTransform(stripSlash(targetFrame),m_BaseFrameID,t);
	} catch (tf2::TransformException &e) {
		ROS_WARN("Failed to lookup local transform %s ",e.what());
		return false;
	}
	return true;
}
