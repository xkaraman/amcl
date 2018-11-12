/*
 * RGBObservationModel.cpp
 *
 *  Created on: Nov 5, 2018
 *      Author: xenakis
 */

#include <RGBObservationModel.h>
#include <MapModel.h>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>

#include <pcl/registration/gicp6d.h>

// Feature Matching
#include <FeatureMatching.h>
//#include <pcl/keypoints/sift_keypoint.h>
//#include <pcl/surface/marching_cubes.h>


//#include <geometry_msgs/Transform.h>
//#include <MapModel.h>
//#include <octomap/ColorOcTree.h>
//#include <pcl/common/common.h>
//#include <RGBObservationModel.h>
//#include <tf2/convert.h>
//#include <tf2_eigen/tf2_eigen.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <pcl/keypoints/narf_keypoint.h>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/gicp6d.h>
//#include <pcl/registration/gicp.h>
//#include <tf2/LinearMath/Transform.h>

#include <pcl/visualization/pcl_visualizer.h>

RGBObservationModel::RGBObservationModel(ros::NodeHandle* nh,
		std::shared_ptr<MapModel> mapModel) {
	m_Map = mapModel->getMap();
	m_BaseToSensorTransform.setIdentity();
	m_RGB = false;
}

RGBObservationModel::~RGBObservationModel() {
	// TODO Auto-generated destructor stub
}


double RGBObservationModel::measure(RobotState const & state) const {
	// transform current particle's pose to its sensor frame
	tf2::Transform particlePose;
	tf2::convert(state.getPose(), particlePose);

	tf2::Transform globalLaserOriginTf = particlePose * m_BaseToSensorTransform;
	geometry_msgs::Transform transformMsg;
	transformMsg = tf2::toMsg(globalLaserOriginTf);
	Eigen::Affine3d tmp = tf2::transformToEigen(transformMsg);

	PointCloudRGB::Ptr pcTransformed = (new PointCloudRGB)->makeShared();
	pcl::transformPointCloud(m_ObservedMeasurementRGB, *pcTransformed, tmp);

	PointRGBT minTransformed,maxTransformed;

//	minTransformed = pcl::transformPoint(m_min3D,tmp);
//	maxTransformed = pcl::transformPoint(m_max3D,tmp);
//
////	double x,y,z;
////	m_Map->getMetricMin(x,y,z);
////	if(minTransformed.x < x) minTransformed.x = x;
////	if(minTransformed.y < y) minTransformed.y = y;
////	if(minTransformed.z < z) minTransformed.z = z;
////	m_Map->getMetricMax(x,y,z);
////	if(maxTransformed.x > x) maxTransformed.x = x;
////	if(maxTransformed.y > y) maxTransformed.y = y;
////	if(maxTransformed.z > z) maxTransformed.z = z;

	pcl::getMinMax3D<PointRGBT>(*pcTransformed,minTransformed,maxTransformed);

	octomap::point3d min(minTransformed.x,minTransformed.y,minTransformed.z);
	octomap::point3d max(maxTransformed.x,maxTransformed.y,maxTransformed.z);

	octomap::ColorOcTree::leaf_bbx_iterator it = m_Map->begin_leafs_bbx(min,max);
	octomap::ColorOcTree::leaf_bbx_iterator end = m_Map->end_leafs_bbx();

	PointCloudRGB::Ptr octoMapPointCloud = (new PointCloudRGB)->makeShared();
	octoMapPointCloud->is_dense = false;
	pcTransformed->is_dense = false;

	for( ; it!=end; ++it ){
		PointRGBT tmp;
		tmp.x = it.getX();
		tmp.y = it.getY();
		tmp.z = it.getZ();
		tmp.r = it->getColor().r;
		tmp.g = it->getColor().g;
		tmp.b = it->getColor().b;
		octoMapPointCloud->push_back(tmp);
	}

//	std::cerr << "pointcloud sensor " << pcTransformed->points.size() << " data points sensor" << std::endl;
//	std::cerr << "pointcloud octomap " << octoMapPointCloud->points.size() << " data points octomap" << std::endl;

//	// show results
//	pcl::visualization::PCLVisualizer viewer("Viewer for original clouds");
//
//	viewer.addPointCloud(octoMapPointCloud, "octoMap");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "octoMap"); // RED
//
//	viewer.addPointCloud(pcTransformed, "sensor");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "sensor"); // BLUE
//
//	while (!viewer.wasStopped ())
//	  {
//		viewer.spinOnce (100);
//		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//	  }

	// Down sample using leaf size of m
	pcl::VoxelGrid<PointRGBT> sor;
	sor.setInputCloud(octoMapPointCloud);
	sor.setLeafSize(0.2f, 0.2f, 0.2f);
	sor.filter(*octoMapPointCloud);

//	std::cerr << "pointcloud downsampled sensor " << pcTransformed->points.size() << " data points sensor" << std::endl;
//	std::cerr << "pointcloud downsampled octomap " << octoMapPointCloud->points.size() << " data points octomap" << std::endl;

	// make dense
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*pcTransformed, *pcTransformed, indices);
	pcl::removeNaNFromPointCloud(*octoMapPointCloud,  *octoMapPointCloud,  indices);

	if (!octoMapPointCloud || octoMapPointCloud->points.size() <= 0)
		return 1/9999.0;

	double fitnessScore;
	if (m_RGB){
		pcl::GeneralizedIterativeClosestPoint6D gicp;
		gicp.setInputSource(pcTransformed);
		gicp.setInputTarget(octoMapPointCloud);
		PointCloudRGB::Ptr transformed(new PointCloudRGB);
		gicp.align(*transformed);
		fitnessScore = gicp.getFitnessScore();
	}
	else {
		pcl::IterativeClosestPoint<PointRGBT, PointRGBT> icp;
		icp.setInputSource(pcTransformed);
		icp.setInputTarget(octoMapPointCloud);
	//	icp.setMaximumIterations(50);
	//	icp.setTransformationEpsilon(1e-8);
		pcl::PointCloud<PointRGBT>::Ptr transformed(new pcl::PointCloud<PointRGBT>);
		icp.align(*transformed);
		fitnessScore = icp.getFitnessScore();
	}




//	std::cerr << "pointcloud transfrom to octomap " << transformed->points.size() << " data points "<< std::endl;

//    // show results
//    pcl::visualization::PCLVisualizer viewer1("Viewer for GICP");
//
//    viewer1.addPointCloud(octoMapPointCloud, "octoMap");
//    viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "octoMap");
//
//    viewer1.addPointCloud(pcTransformed, "sensor");
//    viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "sensor");
//
//    viewer1.addPointCloud(transformed, "transformed");
//    viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "transformed");
//
//    while (!viewer1.wasStopped ())
//      {
//        viewer1.spinOnce (100);
//        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//      }

//    std::cerr << gicp.getFitnessScore();
	return 1/fitnessScore;

//	std::string name = "sensor.pcd";
//	pcl::io::savePCDFileBinaryCompressed(name, *pcTransformed);

//	name = "sensor" + std::to_string(rand()) + ".pcd";
//	pcl::io::savePCDFileBinaryCompressed (name, *octoMapPointCloud);
//	std::cerr << "Saved " << octoMapPointCloud->points.size() << " data points sensor" << std::endl;

	boost::shared_ptr<pcl::Keypoint<PointRGBT, pcl::PointXYZI> > keypoint_detector;
	pcl::SIFTKeypoint<PointRGBT, pcl::PointXYZI>* sift3D = new pcl::SIFTKeypoint<PointRGBT, pcl::PointXYZI>;
	sift3D->setScales (0.01f, 3, 2);
	sift3D->setMinimumContrast (0.0);
	keypoint_detector.reset (sift3D);

	boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstruction;
	pcl::MarchingCubes<pcl::PointXYZRGBNormal>* mc = new pcl::MarchingCubesHoppe<pcl::PointXYZRGBNormal>;
	mc->setIsoLevel (0.001f);
	mc->setGridResolution (50, 50, 50);
	surface_reconstruction.reset(mc);

	pcl::Feature<PointRGBT, pcl::PFHRGBSignature250>::Ptr feature_extractor (new pcl::PFHRGBEstimation<PointRGBT, pcl::Normal, pcl::PFHRGBSignature250>);
	feature_extractor->setKSearch(60);
	FeatureMatching<pcl::PFHRGBSignature250> tutorial(keypoint_detector, feature_extractor, surface_reconstruction, pcTransformed, octoMapPointCloud);

//	pcl::SHOTColorEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT1344>* shot = new pcl::SHOTColorEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT1344>;
//    shot->setRadiusSearch (0.04);
//	pcl::Feature<pcl::PointXYZRGBA, pcl::SHOT1344>::Ptr feature_extractor (shot);
//	FeatureMatching<pcl::SHOT1344> tutorial(keypoint_detector, feature_extractor, surface_reconstruction, pcTransformed, octoMapPointCloud);

	tutorial.match();
//		while(!tutorial.visualizer_.wasStopped())
//		tutorial.run();

	return 1 / tutorial.getFitnessScore();

}

void RGBObservationModel::setMap(
		std::shared_ptr<octomap::ColorOcTree> const & map) {
	m_Map = map;
}

void RGBObservationModel::setBaseToSensorTransform(
		tf2::Transform const & baseToSensor) {
	m_BaseToSensorTransform = baseToSensor;
}

void RGBObservationModel::setObservedMeasurements(
		PointCloudRGB const & observed) {
	m_ObservedMeasurementRGB = observed;
}

void RGBObservationModel::setMinMax3D(PointRGBT min, PointRGBT max) {
	m_min3D = min;
	m_max3D = max;
}

void RGBObservationModel::setRGB(bool const rgb) {
	m_RGB = rgb;
}
