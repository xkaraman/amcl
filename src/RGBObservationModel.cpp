/*
 * RGBObservationModel.cpp
 *
 *  Created on: Nov 5, 2018
 *      Author: xenakis
 */

#include <FeatureMatching1.h>
#include <RGBObservationModel.h>
#include <MapModel.h>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/keypoints/iss_3d.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>

// Feature Matching
#include <pcl/visualization/pcl_visualizer.h>

RGBObservationModel::RGBObservationModel(ros::NodeHandle* nh,
		std::shared_ptr<MapModel> mapModel)
:m_ObservedMeasurementRGB(new PointCloudRGB){
	m_Map = mapModel->getMap();
	m_BaseToSensorTransform.setIdentity();
	m_RGB = false;
	m_NodeHandle = nh;
	nh->param("obs/keypoint_type",m_KeypointType,1);
	nh->param("obs/descriptor_type",m_DescriptorType,1);
	nh->param("downsample_voxel_size",m_DownsampleVoxelSize,0.3f);
	nh->param("obs/maximum_icp_iterations",m_MaximumIterations,50);
	nh->param("obs/transformation_epsilon",m_TransformationEpsilon,1e-8);
	nh->param("obs/octomap_leaf_depth",m_OctomapDepth,16);
}

RGBObservationModel::~RGBObservationModel() {
	// TODO Auto-generated destructor stub
}


double RGBObservationModel::measure(RobotState const & state) const {
	// transform current particle's pose to its sensor frame
	tf2::Transform particlePose;
	tf2::convert(state.getPose(), particlePose);

	octomap::point3d pose(particlePose.getOrigin().getX(),particlePose.getOrigin().getY(),particlePose.getOrigin().getZ());

	tf2::Transform globalLaserOriginTf = particlePose * m_BaseToSensorTransform;
	geometry_msgs::Transform transformMsg;
	transformMsg = tf2::toMsg(globalLaserOriginTf);
	Eigen::Affine3d tmp = tf2::transformToEigen(transformMsg);

//	std::cout << "Transformation Matrix: " << transformMsg << std::endl;
//	std::cout << "Original Measurement: " << m_ObservedMeasurementRGB << std::endl;

//	PointRGBT minoriginal,maxoriginal;
//	pcl::getMinMax3D<PointRGBT>(*m_ObservedMeasurementRGB,minoriginal,maxoriginal);

//	std::cout<<"Min Point at: " << minoriginal << std::endl;
//	std::cout<<"Max Point at: " << maxoriginal << std::endl;

	PointCloudRGB::Ptr pcTransformed(new PointCloudRGB());
	pcl::transformPointCloud(*m_ObservedMeasurementRGB, *pcTransformed, tmp);

	PointRGBT minTransformed,maxTransformed;
	pcl::getMinMax3D<PointRGBT>(*pcTransformed,minTransformed,maxTransformed);
//	minTransformed = pcl::transformPoint(m_min3D,tmp);
//	maxTransformed = pcl::transformPoint(m_max3D,tmp);

//	std::cout<<"Min Point Transformed at: " << minTransformed << std::endl;
//	std::cout<<"Max Point Transformed at: " << maxTransformed << std::endl;

	octomap::point3d min(minTransformed.x,minTransformed.y,minTransformed.z);
	octomap::point3d max(maxTransformed.x,maxTransformed.y,maxTransformed.z);

	octomap::OcTreeKey minKey;
	octomap::OcTreeKey maxKey;

//	bool inmap;
//	inmap = m_Map->coordToKeyChecked(min,minKey);
//	if(!inmap){
//		ROS_WARN("Out of Bounds min");
//	}
//	inmap = m_Map->coordToKeyChecked(max,maxKey);
//	if(!inmap){
//		ROS_WARN("Out of Bounds max");
//	}
//
//	octomap::ColorOcTree::leaf_bbx_iterator it = m_Map->begin_leafs_bbx(minKey,maxKey,m_OctomapDepth);
//	octomap::ColorOcTree::leaf_bbx_iterator end = m_Map->end_leafs_bbx();

//	octomap::ColorOcTree::leaf_iterator it = m_Map->begin_leafs();
//	octomap::ColorOcTree::leaf_iterator end = m_Map->end_leafs();

	PointCloudRGB::Ptr octoMapPointCloud(new PointCloudRGB);
//	octoMapPointCloud->is_dense = false;
//	pcTransformed->is_dense = false;
//	for( ; it!=end; ++it ){
//		PointRGBT tmp;
//		tmp.x = it.getX();
//		tmp.y = it.getY();
//		tmp.z = it.getZ();
//		tmp.r = it->getColor().r;
//		tmp.g = it->getColor().g;
//		tmp.b = it->getColor().b;
//		octoMapPointCloud->push_back(tmp);
//	}

//	std::cout << "DownSampled cloud: " << pcDownsampled << std::endl;
//	std::cout << "Transformed cloud: " << *pcTransformed << std::endl;
//	std::cout << "Octomap cloud    : " << *octoMapPointCloud << std::endl;

//	Passthough
	pcl::IndicesPtr indices_x(new std::vector<int>);
	pcl::PassThrough<PointRGBT> pass;
	pass.setInputCloud(m_ScenePointCloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(minTransformed.x,maxTransformed.x);
	pass.filter (*indices_x);
	pass.setIndices(indices_x);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(minTransformed.y,maxTransformed.y);
	pass.filter (*indices_x);
	pass.setIndices(indices_x);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(minTransformed.z,maxTransformed.z);
	pass.filter (*octoMapPointCloud);

//	std::cerr << "Sensor DownSampled Cloud: " << *pcTransformed << std::endl;
//	std::cout << "Octomap DownSampled cloud    : " << *octoMapPointCloud << std::endl;

	// make dense
	std::vector<int> indices;
	// Sensor Pointcloud already dense from AMCLDepth call
	pcl::removeNaNFromPointCloud(*pcTransformed, *pcTransformed, indices);
	pcl::removeNaNFromPointCloud(*octoMapPointCloud,  *octoMapPointCloud,  indices);

	if (!octoMapPointCloud || octoMapPointCloud->points.size() <= 20 ||
		!pcTransformed || pcTransformed->points.size() <= 20){
		ROS_INFO("Octomap/Sensor Cloud Points Not Enough");
		// Coherence return < 0.
		// 0 is no coherence
		// towards -inf is very coherent
		return -0.05;
	}


	//----------- Visualize Pointclouds -----------//
//	octomap::ColorOcTree::leaf_iterator itleaf= m_Map->begin_leafs();
//	octomap::ColorOcTree::leaf_iterator endleaf = m_Map->end_leafs();
//	PointCloudRGB::Ptr octoMapFullPointCloud(new PointCloudRGB);
//	for( ; itleaf!=endleaf; ++itleaf ){
//			PointRGBT tmp;
//			tmp.x = itleaf.getX();
//			tmp.y = itleaf.getY();
//			tmp.z = itleaf.getZ();
//			tmp.r = itleaf->getColor().r;
//			tmp.g = itleaf->getColor().g;
//			tmp.b = itleaf->getColor().b;
//			octoMapFullPointCloud->push_back(tmp);
//	}
//
//	PointCloudRGB::Ptr minMaxPointCloud(new PointCloudRGB);
//	minMaxPointCloud->push_back(minTransformed);
//	minMaxPointCloud->push_back(maxTransformed);
//
//	pcl::visualization::PCLVisualizer viewer("Viewer for original clouds");
//
//	viewer.addCoordinateSystem(1.0);
//	viewer.initCameraParameters();
//
////	viewer.addPointCloud(octoMapFullPointCloud, "octomap");
////	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "octomap"); // Green
//
//	viewer.addPointCloud(octoMapPointCloud, "particle");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "particle"); // RED
//
////	viewer.addPointCloud(minMaxPointCloud,"minmax");
////	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"minmax");
////	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 175, 0, "minmax");
//
//	viewer.addPointCloud(pcTransformed, "sensor");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "sensor"); // BLUE
//
//	while (!viewer.wasStopped ())
//	  {
//		viewer.spinOnce (100);
//		boost::this_thread::sleep(boost::posix_time::microseconds (100000));
//	  }

	//==================================//

	FeatureMatching1 feature1(m_NodeHandle,pcTransformed,m_ScenePointCloud);
	feature1.setRGB(m_RGB);
	feature1.match();
//	while(!feature1.visualizer_.wasStopped())
//		feature1.run();
//	return std::exp(-feature1.getCoherence());
	return feature1.getCoherence();
	//-------------------------------------------------------------//



//	//-------- ICP-PoinctCloud -------//
//	double fitnessScore;
//	Eigen::Matrix4f transformation;
//	bool converged;
//
//	FeatureMatching1 feature(m_NodeHandle, pcTransformed,octoMapPointCloud);
//	feature.setRGB(m_RGB);
//	feature.match();
////	while(!feature.visualizer_.wasStopped())
////		feature.run();
//	fitnessScore = feature.getFitnessScore();
//	transformation = feature.getFinalTransformation();
//	converged = feature.hasConverged();
//
////	std:: cout << "Has Converged: " << converged << std::endl;
//
//	Eigen::Vector3f translation;
//	translation = transformation.block<3,1>(0,3);
//
////	std::cout << "Translation" << std::endl;
////	std::cout << translation << std::endl;
//
//	Eigen::Vector3f rotation;
//	Eigen::Matrix3f rot;
//	rot = transformation.block<3,3>(0,0);
//	rotation= rot.eulerAngles(0,1,2);
//
////	std::cout << "Rotation Matrix| RPY | Yaw" << std::endl;
////	std::cout << rot << " | " << rotation << " | "<< rotation[2] << std::endl;
//
////	std::cout << "Translation norm | Abs(Yaw) | FitnessScore" << std::endl;
////	std::cout << translation.norm() << " | " << std::abs(rotation[2]) << " | " << fitnessScore << std::endl;
//
////	double sum = translation.norm() + std::abs((double)rotation[2]) + fitnessScore;
//
////	double weight = 0.5 * (double)translation.norm()/sum + 0.1 * std::abs((double)rotation[2])/sum + 0.8 * fitnessScore/sum;
////	double weight = 0.5 * (double)translation.norm() + 0.1 * std::abs((double)rotation[2]) + 0.8 * fitnessScore;
////	weight = std::exp(-weight);
//
//	double weight = 0.6 * 1 / (std::sqrt(2 * M_PI * 0.1 * 0.1))
//						* std::exp(-(fitnessScore * fitnessScore) / (2 * 0.1 * 0.1));
//
//	weight += 0.4 * 0.2 * std::exp(- 0.2 * translation.norm());
////	double weight = std::exp(-fitnessScore);
////	std::cout << "Weight: " << weight << std::endl;
//
//	ROS_ASSERT( weight >= 0);
////	return 1 / weight;
//	return weight;
	//------------------------------------//

	///----------------Feature-Matching--------------------///
//	boost::shared_ptr<pcl::Keypoint<PointRGBT, pcl::PointXYZI> > keypoint_detector;
//	if (m_KeypointType == 1)
//	{
//		pcl::SIFTKeypoint<pcl::PointXYZRGBA, pcl::PointXYZI>* sift3D = new pcl::SIFTKeypoint<pcl::PointXYZRGBA, pcl::PointXYZI>;
//	    sift3D->setScales (0.01f, 3, 2);
//	    sift3D->setMinimumContrast (0.0);
//	    keypoint_detector.reset (sift3D);
//	} else if (m_KeypointType == 3) {
//		pcl::ISSKeypoint3D<PointRGBT, pcl::PointXYZI> *iss3D = new pcl::ISSKeypoint3D<PointRGBT,pcl::PointXYZI>;
//		double iss_salient_radius_;
//		double iss_non_max_radius_;
//		double iss_gamma_21_ (0.975);
//		double iss_gamma_32_ (0.975);
//		double iss_min_neighbors_ (5);
//		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
//
//		iss_salient_radius_ = 6 * m_DownsampleVoxelSize;
//		iss_non_max_radius_ = 4 * m_DownsampleVoxelSize;
//		iss3D->setSearchMethod(tree);
//		iss3D->setSalientRadius(iss_salient_radius_);
//		iss3D->setNonMaxRadius (iss_non_max_radius_);
//		iss3D->setThreshold21 (iss_gamma_21_);
//		iss3D->setThreshold32 (iss_gamma_32_);
//		iss3D->setMinNeighbors (iss_min_neighbors_);
//		iss3D->setNumberOfThreads (0);
//		keypoint_detector.reset(iss3D);
//	}
//	else
//	{
//		pcl::HarrisKeypoint3D<pcl::PointXYZRGBA,pcl::PointXYZI>* harris3D = new pcl::HarrisKeypoint3D<pcl::PointXYZRGBA,pcl::PointXYZI> (pcl::HarrisKeypoint3D<pcl::PointXYZRGBA,pcl::PointXYZI>::HARRIS);
//	    harris3D->setNonMaxSupression (true);
//	    harris3D->setRadius (0.13f);
//	    harris3D->setRadiusSearch (0.13f);
//	    keypoint_detector.reset (harris3D);
//	    switch (m_KeypointType)
//	    {
//	      case 2:
//	        harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGBA,pcl::PointXYZI>::CURVATURE);
//	      break;
//	    }
//	}
//
//	  switch (m_DescriptorType)
//	  {
//	    case 1:
//	    {
//	      pcl::Feature<pcl::PointXYZRGBA, pcl::FPFHSignature33>::Ptr feature_extractor (new pcl::FPFHEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33>);
//	      feature_extractor->setSearchMethod (pcl::search::Search<pcl::PointXYZRGBA>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGBA>));
//	      feature_extractor->setRadiusSearch (0.15);
////	      std::cout << "pcTransformed: " << *pcTransformed << std::endl;
//	      FeatureMatching<pcl::FPFHSignature33> tutorial (keypoint_detector, feature_extractor, pcTransformed, octoMapPointCloud);
//	      tutorial.match ();
////	      std::cerr << "Number of Matches: " << tutorial.getCorrespondencesSize() << std::endl;
//	      std::cerr << "Fitness Score: " << tutorial.getFitnessScore() << std::endl;
//
//	      while(!tutorial.visualizer_->wasStopped())
//	      		tutorial.run();
//	      return 1 / tutorial.getFitnessScore();
//	    }
//	    break;
//
//	    case 2:
//	    {
//	      pcl::SHOTColorEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT1344>* shot = new pcl::SHOTColorEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT1344>;
//	      shot->setRadiusSearch (0.02);
//	      pcl::Feature<pcl::PointXYZRGBA, pcl::SHOT1344>::Ptr feature_extractor (shot);
////	      std::cout << "pcTransformed: " << *pcTransformed << std::endl;
//	      FeatureMatching<pcl::SHOT1344> tutorial (keypoint_detector, feature_extractor, pcTransformed, octoMapPointCloud);
//	      tutorial.match ();
////	      std::cerr << "Number of Matches: " << tutorial.getCorrespondencesSize() << std::endl;
//	      std::cerr << "Fitness Score: " << tutorial.getFitnessScore() << std::endl;
//
//	      while(!tutorial.visualizer_->wasStopped())
//	    	  tutorial.run();
//	      return 1 / tutorial.getFitnessScore();
//	    }
//	    break;
//
//	    case 3:
//	    {
//	      pcl::Feature<pcl::PointXYZRGBA, pcl::PFHRGBSignature250>::Ptr feature_extractor (new pcl::PFHRGBEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PFHRGBSignature250>);
//	      feature_extractor->setKSearch(50);
//	      FeatureMatching<pcl::PFHRGBSignature250> tutorial (keypoint_detector, feature_extractor, pcTransformed, octoMapPointCloud);
//	      tutorial.match ();
////	      std::cerr << "Number of Matches: " << tutorial.getCorrespondencesSize() << std::endl;
//	      std::cerr << "Fitness Score: " << tutorial.getFitnessScore() << std::endl;
//	      while(!tutorial.visualizer_->wasStopped())
//	      		tutorial.run();
//	      return 1 / tutorial.getFitnessScore();
//	    }
//	    break;
//	}
//
//	tutorial.match();
//		while(!tutorial.visualizer_.wasStopped())
//		tutorial.run();
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
		PointCloudRGB::Ptr observed) {
	m_ObservedMeasurementRGB = observed;
	m_ObservedMeasurementRGB->is_dense = false;
}

void RGBObservationModel::setMinMax3D(const PointRGBT &min,const PointRGBT &max) {
	m_min3D = min;
	m_max3D = max;
}

void RGBObservationModel::setRGB(bool const &rgb) {
	m_RGB = rgb;
}

void RGBObservationModel::setScenePointCloud(PointCloudRGB::Ptr scene) {
	m_ScenePointCloud = scene;
}
