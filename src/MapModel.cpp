/*
 * MapModel.cpp
 *
 *  Created on: Aug 24, 2018
 *      Author: xenakis
 */

#include "MapModel.h"

#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>

#include <iterator>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>

/** Map Model Functions */
MapModel::MapModel(ros::NodeHandle *nh) :
		mMotionObstacleDist(0.2) {
	//TODO Set parameters on ParameterServer
}

MapModel::~MapModel() {

}

std::shared_ptr<octomap::ColorOcTree> MapModel::getMap() const {
	return mMap;
}

//void MapModel::initGlobal(Particles &particles, Eigen::Vector3d const &initNoise,UniformDistrT &rndUniform,NormalDistrT &rndNormal) {
//	double sizeX, sizeY, sizeZ, minX, minY, minZ;
//
//	ROS_INFO("Initializing for Global Localization");
//	mMap->getMetricSize(sizeX, sizeY, sizeZ);
//	mMap->getMetricMin(minX, minY, minZ);
//
//	RandomEngineT rndEngine;
//
//	double weight = 1.0 / particles.size();
//
//	Particles::iterator it;
//
//	for (it = particles.begin(); it < particles.end(); it++) {
//		double x,y;
//		x = minX + sizeX * rndUniform(rndEngine);
//		y = minY + sizeY * rndUniform(rndEngine);
//
//		it->pose.position.x = x; // getOrigin().setX(x);
//		it->pose.position.y = y; //getOrigin().setY(y);
//
//		it->pose.position.z = 0.0; //getOrigin().setZ(0.0); // Z unnessecary
//
//		double yaw = rndUniform(rndEngine) * 2 * M_PI - M_PI;
//		tf2::Quaternion q;
//		q.setEuler(yaw,0.0,0.0);
//		convert(q,it->pose.orientation);
//		it->weight = weight;
//	}
//
//	ROS_INFO("Initialization Done");
//}

bool MapModel::isOccupied(const octomap::point3d& position) const{
	octomap::OcTreeNode *ocNode = mMap->search(position);
	if (ocNode) {
		return isOccupied(ocNode);
	}
	else
		return false;
}

/* Occupancy Grid Map */
OccupancyMap::OccupancyMap(ros::NodeHandle* nh) : MapModel(nh) {
	std::string srvName = "octomap_full";
	ROS_INFO("Requesting the map from %s...", nh->resolveName(srvName).c_str());
	octomap_msgs::GetOctomap::Request req;
	octomap_msgs::GetOctomap::Response resp;

	while (nh->ok() && !ros::service::call(srvName,req,resp)){
		ROS_WARN("Request to %s failed; trying again...", nh->resolveName(srvName).c_str());
		usleep(1000000);
	}


	octomap::ColorOcTree* colorTree = dynamic_cast<octomap::ColorOcTree*>(octomap_msgs::fullMsgToMap(resp.map));

	mMap.reset(colorTree);
	if(!mMap && (mMap->size() <= 1) ){
		ROS_ERROR("Map didn't retrieved,exiting");
		exit(-1);
	}
	double x,y,z;
	mMap->getMetricSize(x,y,z);
	ROS_INFO("Occupancy map initialized with %zd nodes (%.2f x %.2f x %.2f m), %f m res.",
											mMap->size(), x,y,z, mMap->getResolution());
}

OccupancyMap::~OccupancyMap() {
}

bool OccupancyMap::isOccupied(octomap::OcTreeNode* node) const {
	return mMap->isNodeOccupied(node);
}
