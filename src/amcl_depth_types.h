/*
 * amcl_depth_types.h
 *
 *  Created on: Nov 8, 2018
 *      Author: xenakis
 */

#ifndef SRC_AMCL_DEPTH_TYPES_H_
#define SRC_AMCL_DEPTH_TYPES_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGBA PointRGBT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointRGBT> PointCloudRGB;



#endif /* SRC_AMCL_DEPTH_TYPES_H_ */
