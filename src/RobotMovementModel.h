#ifndef CARMOVEMENTMODEL_H
#define CARMOVEMENTMODEL_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <libPF/MovementModel.h>
//#include <ros/time.h>
#include <string>
#include "RobotState.h"

namespace tf2_ros {
class TransformListener;
class Buffer;
} /* namespace tf2_ros */

namespace ros {
class NodeHandle;
} /* namespace ros */

namespace libPF {
class RandomNumberGenerationStrategy;
} /* namespace libPF */


/** 
 * @class RobotMovementModel
 */
class RobotMovementModel: public libPF::MovementModel<RobotState> {

public:
	/**
	 * Constructor
	 */
	RobotMovementModel(ros::NodeHandle *nh, tf2_ros::Buffer *tfBuffer,
			const std::string &odomFrameID, const std::string &baseFrameID);

	/**
	 * Destructor
	 */
	~RobotMovementModel();

	/**
	 * The drift method propagates the car using its speed.
	 * @param state Pointer to the state that has to be manipulated.
	 */
	void drift(RobotState& state, double dt) const;

	/**
	 * The diffusion consists of a very small gaussian jitter on the
	 * state's variable.
	 * @param state Pointer to the state that has to be manipulated.
	 */
	void diffuse(RobotState& state, double dt) const;

	/**
	 * @param d new standard deviation for the diffusion of x
	 */
	void setXStdDev(double d);

	/**
	 * @return the standard deviation for the diffusion of x
	 */
	double getXStdDev() const;

	/**
	 * @param d new standard deviation for the diffusion of y
	 */
	void setYStdDev(double d);

	/**
	 * @return the standard deviation for the diffusion of y
	 */
	double getYStdDev() const;

	/**
	 * @param d new standard deviation for the diffusion of theta
	 */
	void setThetaStdDev(double d);

	/**
	 * @return the standard deviation for the diffusion of theta
	 */
	double getThetaStdDev() const;

	/**
	 * Reset the MovementModel
	 */
	void reset();

	/**
	 * Find the Odomotry pose at at a time t
	 */
	bool lookupOdomPose(const ros::Time &t,
			geometry_msgs::PoseStamped &odomPose) const;

	/**
	 * Find the Transform beetwen the odometry pose at t and last saved odometry pose
	 * Calls computeOdomTransform
	 */
	bool lookupOdomTransform(const ros::Time &t,
			geometry_msgs::TransformStamped &odomTransform) const;

	/**
	 * Compute the transform between currentPose and last saved odometry pose
	 */
	geometry_msgs::TransformStamped computeOdomTransform(geometry_msgs::PoseStamped const &currentPose) const;

	/**
	 * Apply the transform to Robot's State
	 */
	void applyOdomTransform(geometry_msgs::Pose &statePose,const geometry_msgs::TransformStamped &odomTransform) const;

	/**
	 * Find the transform between base->target in TF Frame tree
	 */
	bool lookupTargetToBaseTransform(std::string const &targetFrame,
			ros::Time const &t, geometry_msgs::TransformStamped &localTransform) const ;

	/**
	 * Set the last odometry pose
	 */
	void setLastOdomPose(geometry_msgs::PoseStamped &pose);

	/**
	 * Get the last saved odomoetry pose
	 */
	geometry_msgs::PoseStamped getLastOdomPose() const;
protected:

private:
	/// Stores the random number generator
	libPF::RandomNumberGenerationStrategy* m_RNG;

	geometry_msgs::PoseStamped m_LastOdomPose;
	bool m_FirstOdometryReceived;

	tf2_ros::Buffer *m_TFBuffer;
	tf2_ros::TransformListener *m_TFListener;

	std::string m_OdomFrameID;
	std::string m_BaseFrameID;

	/// Store the standard deviations of the model
	double m_XStdDev;
	double m_YStdDev;
	double m_ThetaStdDev;
};

#endif

