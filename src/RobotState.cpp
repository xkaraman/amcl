#include "RobotState.h"

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <cmath>

RobotState::RobotState() :
m_Pose()
{
}

RobotState::~RobotState()
{
}

RobotState RobotState::operator*(float factor) const
{
    RobotState newState;
    tf2::Transform tf;

    tf2::fromMsg(m_Pose,tf);

    tf.getOrigin() = tf.getOrigin() * factor;

    tf2::toMsg(tf,newState.m_Pose);

    return newState;
}

RobotState& RobotState::operator+=(const RobotState& other)
{
	tf2::Transform tfThis;
	tf2::convert(m_Pose,tfThis);

	tf2::Transform tfOther;
	tf2::convert(other.m_Pose,tfOther);

	tfThis.getOrigin() += tfOther.getOrigin();
	tfThis.getBasis()[0] += tfOther.getBasis()[0];
	tfThis.getBasis()[1] += tfOther.getBasis()[1];
	tfThis.getBasis()[2] += tfOther.getBasis()[2];

	geometry_msgs::Pose result;
	tf2::toMsg(tfThis,result);
	this->setPose(result);

	return *this;
}

RobotState& RobotState::operator=(const RobotState& other){
	this->m_Pose = other.m_Pose;
	return *this;
}

float RobotState::getXPos() const
{
    return m_Pose.position.x;
}

void RobotState::setXPos(float x)
{
	m_Pose.position.x = x;
}

float RobotState::getYPos() const
{
    return m_Pose.position.y;
}

void RobotState::setYPos(float y)
{
	m_Pose.position.y = y;
}

float RobotState::getTheta() const
{
	tf2::Quaternion q;
	tf2::convert(m_Pose.orientation,q);
    return tf2::getYaw(q);
}

void RobotState::setTheta(float t)
{
	tf2::Quaternion q;
	q.setRPY(0.0,0.0,t);
	tf2::convert(q,m_Pose.orientation);
}

geometry_msgs::Pose RobotState::getPose() const {
	return m_Pose;
}

void RobotState::setPose(geometry_msgs::Pose p) {
	m_Pose = p;
}
