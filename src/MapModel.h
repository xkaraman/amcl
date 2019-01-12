/*
 * MapModel.h
 *
 *  Created on: Aug 24, 2018
 *      Author: xenakis
 */

#ifndef INCLUDE_MAPMODEL_H_
#define INCLUDE_MAPMODEL_H_

#include <amcl_depth_types.h>
#include <octomap/ColorOcTree.h>

#include <octomap/octomap_types.h>
#include <memory>

namespace octomap {
//class ColorOcTree;
class OcTreeNode;
} /* namespace octomap */
namespace ros {
class NodeHandle;
} /* namespace ros */

/*
 *
 */
class MapModel {
public:
	/**
	 * Constructor
	 */
	MapModel(ros::NodeHandle *nh);

	/**
	 * Deconstructor
	 */
	virtual ~MapModel();

	/**
	 * Return Map in current Model
	 */
	std::shared_ptr<octomap::ColorOcTree> getMap() const;

	//virtual void initGlobal(Particles &particles, Vector3d const &initNoise,UniformDistrT &rndUniform,NormalDistrT &rndNormal);

	/**
	 * Check whether a point is occupied in OctoMap structure
	 */
	virtual bool isOccupied(const octomap::point3d& position) const;

	/**
	 * Check whether a OcTree node is occupied in OctoMap structure
	 */
	virtual bool isOccupied(octomap::OcTreeNode *node) const = 0;

protected:
	std::shared_ptr<octomap::ColorOcTree> mMap;
	double mMotionObstacleDist;
};

class OccupancyMap: public MapModel {
public:
	OccupancyMap(ros::NodeHandle *nh);
	virtual ~OccupancyMap();

	bool isOccupied(octomap::OcTreeNode *node) const;

	PointCloudRGB toPCL();
};

#endif
