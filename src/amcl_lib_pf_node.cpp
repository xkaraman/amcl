#include <ros/ros.h>
#include <AMCLDepth.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "amcl_depth");

	ROS_INFO("Initializing node");

	AMCLDepth amcl;

	ros::spin();

	return (0);
}

