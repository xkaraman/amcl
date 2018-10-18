#ifndef CARSTATE_H
#define CARSTATE_H

#include <geometry_msgs/Pose.h>

/**
 * @class RobotState
 * @brief Sample state for a particle filter that simulates a Turlebot2.
 *
 * This state has the following parameters:
 * @li <b>pose</b> the overall Pose of the robot
 * Infered:
 * @li <b>xpos</b> the absolute x-Position of the robot (map frame)
 * @li <b>ypos</b> the y-Position of the robot (map frame)
 * @li <b>theta</b> the orientation of the robot (in radiants) (map frame)

 */
class RobotState
{
public:
    RobotState();
    ~RobotState();

    RobotState operator*(float factor) const;

    RobotState& operator+=(const RobotState& other);

    RobotState& operator=(const RobotState& other);

    float getXPos() const;

    void setXPos(float x);

    float getYPos() const;

    void setYPos(float y);

    float getTheta() const;

    void setTheta(float t);

    geometry_msgs::Pose getPose() const;

    void setPose(geometry_msgs::Pose p);

private:
    geometry_msgs::Pose m_Pose;
};

#endif // CARSTATE_H
