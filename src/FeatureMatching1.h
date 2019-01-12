/*
 * FeatureMatching.h
 *
 *  Created on: Oct 29, 2018
 *      Author: xenakis
 */

#ifndef SRC_FEATUREMATCHING1_H_
#define SRC_FEATUREMATCHING1_H_

#include <amcl_depth_types.h>

#include <ros/ros.h>

#include <vector>
#include <string>
#include <sstream>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/angles.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/shot_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>


#include <pcl/tracking/coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>

class FeatureMatching1
{
  public:
    FeatureMatching1 (ros::NodeHandle* nh, PointCloudRGB::ConstPtr source,
                  	 PointCloudRGB::ConstPtr target);

    void match();
    /**
     * @brief starts the event loop for the visualizer
     */
//    void run ();
//    pcl::visualization::PCLVisualizer visualizer_;

    Eigen::Matrix4f getFinalTransformation() const;

    double getFitnessScore() const;

    float getCoherence() const;

    bool hasConverged() const;

    void setRGB(const bool &rgb);

  protected:
    /**
     * @brief remove plane and select largest cluster as input object
     * @param input the input point cloud
     * @param segmented the resulting segmented point cloud containing only points of the largest cluster
     */
    void segmentation(PointCloudRGB::ConstPtr input, PointCloudRGB::Ptr segmented) const;

    void detectKeypoints(PointCloudRGB::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints) const;

    void extractDescriptors(PointCloudRGB::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features);

    void findCorrespondences (pcl::PointCloud<pcl::FPFHSignature33>::Ptr source,pcl::PointCloud<pcl::FPFHSignature33>::Ptr target, std::vector<int>& correspondences) const;

    void filterCorrespondences ();

    void determineInitialTransformation ();

    void findCoherence(PointCloudRGB::ConstPtr source, PointCloudRGB::ConstPtr target,float &w);
    /**
     * @brief calculate the final rigid transformation using ICP over all points
     */
    void determineFinalTransformation ();

    /**
     * @brief callback to handle keyboard events
     * @param event object containing information about the event. e.g. type (press, release) etc.
     * @param cookie user defined data passed during registration of the callback
     */
    void keyboard_callback (const pcl::visualization::KeyboardEvent& event, void* cookie);

  private:
    PointCloudRGB::ConstPtr source_;
    PointCloudRGB::ConstPtr target_;
    PointCloudRGB::Ptr source_segmented_;
    PointCloudRGB::Ptr target_segmented_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr  source_features_;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr  target_features_;
    std::vector<int> source2target_;
    std::vector<int> target2source_;
    pcl::CorrespondencesPtr correspondences_;
    PointCloudRGB::Ptr source_transformed_;
    PointCloudRGB::Ptr source_registered_;
    Eigen::Matrix4f initial_transformation_matrix_;
    Eigen::Matrix4f transformation_matrix_;
    bool show_source2target_;
    bool show_target2source_;
    bool show_correspondences;
    bool rgb_;
    double fitness_score_;
    float coherence_;
	bool converged_;
	double m_MaximumDistance;
};

inline FeatureMatching1::FeatureMatching1(ros::NodeHandle* nh, PointCloudRGB::ConstPtr source,
		PointCloudRGB::ConstPtr target)
:source_ (source)
,target_ (target)
,source_segmented_ (new PointCloudRGB)
,target_segmented_ (new PointCloudRGB)
,source_transformed_(new PointCloudRGB)
,source_registered_(new PointCloudRGB)
,source_keypoints_(new pcl::PointCloud<pcl::PointXYZI>)
,target_keypoints_(new pcl::PointCloud<pcl::PointXYZI>)
,source_features_ (new pcl::PointCloud<pcl::FPFHSignature33>)
,target_features_ (new pcl::PointCloud<pcl::FPFHSignature33>)
,correspondences_ (new pcl::Correspondences)
,show_source2target_ (false)
,show_target2source_ (false)
,show_correspondences (false)
,fitness_score_(0)
,coherence_(0)
,rgb_(false)
,converged_(false)
{
//  visualizer_.registerKeyboardCallback(&FeatureMatching1::keyboard_callback, *this, 0);
	nh->param("coherence/maximum_distance",m_MaximumDistance,0.01);

}

inline void FeatureMatching1::setRGB(const bool &rgb){
	rgb_ = rgb;
}


inline void FeatureMatching1::match() {
	segmentation (source_, source_segmented_);
	segmentation (target_, target_segmented_);

	findCoherence(source_segmented_,target_segmented_,coherence_);

//	detectKeypoints (source_segmented_, source_keypoints_);
//	detectKeypoints (target_segmented_, target_keypoints_);
//
//	extractDescriptors (source_segmented_, source_keypoints_, source_features_);
//	extractDescriptors (target_segmented_, target_keypoints_, target_features_);

//	findCorrespondences (source_features_, target_features_, source2target_);
//	findCorrespondences (target_features_, source_features_, target2source_);
//
//	filterCorrespondences ();
//

//	if(target_segmented_->size() < 20 ){
////		std::cout << "Feature Matching target_seg was empty: Copying " << std::endl;
////		std::cout << "Feature Matching target_seg before: " << *target_segmented_ << std::endl;
//		pcl::copyPointCloud(*target_,*target_segmented_);
////		std::cout << "Feature Matching target_seg after: " << *target_segmented_ << std::endl;
//	}
//
//	if(source_segmented_->size() < 20 ){
////		std::cout << "Feature Matching source_segmented_ was empty: Copying " << std::endl;
////		std::cout << "Feature Matching source_segmented_ before: " << *source_segmented_ << std::endl;
//		pcl::copyPointCloud(*source_,*source_segmented_);
////		std::cout << "Feature Matching source_segmented_ after: " << *source_segmented_ << std::endl;
//	}
//////	determineInitialTransformation();
//	determineFinalTransformation();

//	std::cout << "Feature Matching source: " << *source_ << std::endl;
//	std::cout << "Feature Matching target: " << *target_ << std::endl;
//	std::cout << "Feature Matching source_seg: " << *source_segmented_ << std::endl;
//	std::cout << "Feature Matching target_seg: " << *target_segmented_ << std::endl;
//	std::cout << "Feature Matching registerd: " << *source_registered_ << std::endl;
}

inline bool FeatureMatching1::hasConverged() const{
	return converged_;
}

inline double FeatureMatching1::getFitnessScore() const {
	return fitness_score_;
}

inline float FeatureMatching1::getCoherence() const {
	return coherence_;
}

inline void FeatureMatching1::findCoherence(PointCloudRGB::ConstPtr source,
		PointCloudRGB::ConstPtr target, float &w) {
	using namespace pcl::tracking;
	    ApproxNearestPairPointCloudCoherence<PointRGBT>::Ptr coherence = ApproxNearestPairPointCloudCoherence<PointRGBT>::Ptr
	(new ApproxNearestPairPointCloudCoherence<PointRGBT> ());

	    boost::shared_ptr<DistanceCoherence<PointRGBT> > distanceCoherence
	    = boost::shared_ptr<DistanceCoherence<PointRGBT> > (new DistanceCoherence<PointRGBT> ());
	    coherence->addPointCoherence(distanceCoherence);

	    if(rgb_){
		 boost::shared_ptr<HSVColorCoherence<PointRGBT> > colorCoherence
		= boost::shared_ptr<HSVColorCoherence<PointRGBT> > (new HSVColorCoherence<PointRGBT> ());
		colorCoherence->setWeight(0.5);
		coherence->addPointCoherence(colorCoherence);
	    }
		boost::shared_ptr<pcl::search::Octree<PointRGBT> > search (new pcl::search::Octree<PointRGBT> (0.01));

	    coherence->setSearchMethod (search);
	    coherence->setMaximumDistance (m_MaximumDistance);

		coherence->setTargetCloud(target);

		pcl::IndicesPtr indice (new std::vector<int>);
		coherence->compute(source,indice,w);

//		std:: cout << "Weight: " << w << " " << exp(w) << std::endl;
}
inline void FeatureMatching1::segmentation(PointCloudRGB::ConstPtr input,
		PointCloudRGB::Ptr segmented) const {
	// cout << "segmentation..." << std::flush;
	  // fit plane and keep points above that plane
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  // Create the segmentation object
	  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	  // Optional
	  seg.setOptimizeCoefficients (true);
	  // Mandatory
	  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setAxis(Eigen::Vector3f(0,0,1));
	  seg.setMaxIterations(50);
	  seg.setEpsAngle(pcl::deg2rad(1.0));
	  seg.setDistanceThreshold(0.1);

	  seg.setInputCloud (input);
	  seg.segment (*inliers, *coefficients);

	  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	  if(inliers->indices.size() > 0 ){
		  extract.setInputCloud (input);
		  extract.setIndices (inliers);
		  extract.setNegative (true);

		  extract.filter (*segmented);

		  std::vector<int> indices;
		  segmented->is_dense = false;
		  pcl::removeNaNFromPointCloud<PointRGBT>(*segmented, *segmented, indices);
		  // cout << "OK" << endl;
	  } else {
		  std::vector<int> indices;
		  segmented->is_dense = false;
		  pcl::removeNaNFromPointCloud<PointRGBT>(*input, *segmented, indices);
		  // cout << "OK" << endl;
	  }
	  // cout << "clustering..." << std::flush;
	//   euclidean clustering
//	  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
//	  tree->setInputCloud (segmented);
//
//	  std::vector<pcl::PointIndices> cluster_indices;
//	  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> clustering;
//	  clustering.setClusterTolerance (0.02); // 2cm
//	  clustering.setMinClusterSize (500);
//	  clustering.setMaxClusterSize (25000);
//	  clustering.setSearchMethod (tree);
//	  clustering.setInputCloud(segmented);
//	  clustering.extract (cluster_indices);
//
//	  if (cluster_indices.size() > 0)//use largest cluster
//	  {
//	    // cout << cluster_indices.size() << " clusters found";
//	    if (cluster_indices.size() > 1)
//	//       cout <<" Using largest one...";
//	     cout << endl;
//
//	    pcl::IndicesPtr indices (new std::vector<int>);
//	    *indices = cluster_indices[0].indices;
//	    extract.setInputCloud (segmented);
//	    extract.setIndices (indices);
//	    extract.setNegative (false);
//
//	    extract.filter (*segmented);
//	  }
}

inline void FeatureMatching1::detectKeypoints(PointCloudRGB::ConstPtr input,
		pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints) const {
//	pcl::SIFTKeypoint<pcl::PointXYZRGBA, pcl::PointXYZI>* sift3D = new pcl::SIFTKeypoint<pcl::PointXYZRGBA, pcl::PointXYZI>;
//	sift3D->setScales (0.01f, 3, 2);
//	sift3D->setMinimumContrast (0.0);
//	sift3D->setInputCloud(input);
//	sift3D->compute(*keypoints);

	pcl::HarrisKeypoint3D<PointRGBT,pcl::PointXYZI> harris;// = new pcl::ISSKeypoint3D<PointRGBT,pcl::PointXYZI>;
	harris.setNonMaxSupression (true);
	harris.setInputCloud (input);
	harris.setThreshold (1e-6);
	harris.setRadius(0.4f);
	harris.compute(*keypoints);

//	pcl::ISSKeypoint3D<PointRGBT, pcl::PointXYZI> *iss3D = new pcl::ISSKeypoint3D<PointRGBT,pcl::PointXYZI>;
//	double iss_salient_radius_;
//	double iss_non_max_radius_;
//	double iss_gamma_21_ (0.975);
//	double iss_gamma_32_ (0.975);
//	double iss_min_neighbors_ (5);
//	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
//
//	iss_salient_radius_ = 3 * 0.1;
//	iss_non_max_radius_ = 2 * 0.1;
//	iss3D->setSearchMethod(tree);
//	iss3D->setSalientRadius(iss_salient_radius_);
//	iss3D->setNonMaxRadius (iss_non_max_radius_);
//	iss3D->setThreshold21 (iss_gamma_21_);
//	iss3D->setThreshold32 (iss_gamma_32_);
//	iss3D->setMinNeighbors (iss_min_neighbors_);
//	iss3D->setNumberOfThreads (0);
//	iss3D->setInputCloud(input);
//	iss3D->compute(*keypoints);
//	cout << "Keypoints Found: " << keypoints->size() << endl;
}

inline void FeatureMatching1::extractDescriptors(PointCloudRGB::ConstPtr input,
		pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr features) {
	PointCloudRGB::Ptr kpts(new PointCloudRGB);
	kpts->points.resize(keypoints->points.size());

//	cout << "Keypoints Size: " << keypoints->size() << endl;
//	cout << "kpts Size: " << kpts->size() << endl;

	pcl::copyPointCloud(*keypoints, *kpts);

	pcl::PointCloud<pcl::Normal>::Ptr normals (new  pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<PointRGBT, pcl::Normal> normal_estimation;
	normal_estimation.setSearchMethod (pcl::search::Search<PointRGBT>::Ptr (new pcl::search::KdTree<PointRGBT>));
	normal_estimation.setKSearch(10);
	normal_estimation.setInputCloud (input);
	normal_estimation.compute (*normals);

//	cout << "Normals Size: " << normals->size() << endl;

	pcl::FPFHEstimationOMP<PointRGBT, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud (kpts);
	fpfh.setSearchSurface(input);
	fpfh.setInputNormals (normals);
	pcl::search::KdTree<PointRGBT>::Ptr tree (new pcl::search::KdTree<PointRGBT>);
	fpfh.setSearchMethod(tree);
	fpfh.setKSearch(10);
	fpfh.compute(*features);
//	cout << "Features Computed: " << features->size() << endl;

}

inline void FeatureMatching1::findCorrespondences(
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr source,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr target,
		std::vector<int>& correspondences) const {
	correspondences.resize (source->size());

//	// Use a KdTree to search for the nearest matches in feature space
//	pcl::KdTreeFLANN<pcl::FPFHSignature33> descriptor_kdtree;
//	descriptor_kdtree.setInputCloud (target);
//
//	// Find the index of the best match for each keypoint, and store it in "correspondences_out"
//	const int k = 1;
//	std::vector<int> k_indices (k);
//	std::vector<float> k_squared_distances (k);
//	for (int i = 0; i < static_cast<int> (source->size ()); ++i)
//	{
//		descriptor_kdtree.nearestKSearch (*source, i, k, k_indices, k_squared_distances);
////		correspondences[i] = k_indices[0];
//
//		if(k==1 && k_squared_distances[0]<0.2)
//			correspondences[i] = k_indices[0];
////		if (k == descriptor_kdtree.nearestKSearch (*source, i, k, k_indices, k_squared_distances) ){
////			correspondences[i] = k_indices[0];
//////			cout << "Found Correspondance: " << correspondences[i] << endl;
////		}
////		else
////			cout<< "No match found" << endl;
//	}
}

inline void FeatureMatching1::filterCorrespondences(){
	correspondences_->resize (source_features_->size());
	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33,pcl::FPFHSignature33>::Ptr ce (new pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33>);
	ce->setInputSource (source_features_);
	ce->setInputTarget (target_features_);
	ce->determineReciprocalCorrespondences(*correspondences_);

//	std::vector<std::pair<unsigned, unsigned> > correspondences;
//	for (unsigned cIdx = 0; cIdx < source2target_.size (); ++cIdx)
//		if (target2source_[source2target_[cIdx]] == static_cast<int> (cIdx))
//			correspondences.push_back(std::make_pair(cIdx, source2target_[cIdx]));
//
//	correspondences_->resize (correspondences.size());
//	for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx)
//	{
//		(*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
//		(*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
//	}
//
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector;
	rejector.setInputSource (source_keypoints_);
	rejector.setInputTarget (target_keypoints_);
	rejector.setInputCorrespondences(correspondences_);
	rejector.getCorrespondences(*correspondences_);
//	cout << "Found " << correspondences_->size() << " Correspondences" << endl;
}


void FeatureMatching1::determineInitialTransformation (){

	pcl::SampleConsensusInitialAlignment<PointRGBT, PointRGBT, pcl::FPFHSignature33> sac_ia;
	PointCloudRGB aligned;
	sac_ia.setNumberOfSamples (3);
	sac_ia.setMinSampleDistance (0.2);
	sac_ia.setCorrespondenceRandomness (2);
	sac_ia.setMaximumIterations (20);
	sac_ia.setInputSource (source_segmented_);
	sac_ia.setInputTarget (target_segmented_);
	sac_ia.setSourceFeatures (source_features_);
	sac_ia.setTargetFeatures (target_features_);
	sac_ia.align (aligned);
	initial_transformation_matrix_ = sac_ia.getFinalTransformation ();
//	  std::cout << "Found Initial transform" << std::endl;

//  pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Ptr transformation_estimation (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI>);
//
//  transformation_estimation->estimateRigidTransformation (*source_keypoints_, *target_keypoints_, *correspondences_, initial_transformation_matrix_);

  pcl::transformPointCloud(*source_segmented_, *source_transformed_, initial_transformation_matrix_);
//  std::cout << "transform done" << std::endl;
}

inline void FeatureMatching1::determineFinalTransformation() {
	pcl::Registration<PointRGBT, PointRGBT>::Ptr registration;

	if(rgb_){
		pcl::GeneralizedIterativeClosestPoint6D *gicp = new pcl::GeneralizedIterativeClosestPoint6D;
		registration.reset(gicp);
	}
	else {
		pcl::IterativeClosestPoint<PointRGBT, PointRGBT> *icp = new pcl::IterativeClosestPoint<PointRGBT, PointRGBT>;
		registration.reset(icp);
	}

//	pcl::registration::CorrespondenceEstimation<PointRGBT,PointRGBT>::Ptr ce (new pcl::registration::CorrespondenceEstimation<PointRGBT,PointRGBT>);
//	ce->setInputSource (source_segmented_);
//	ce->setInputTarget (target_segmented_);

//	registration->setInputSource (source_transformed_);
	registration->setInputSource (source_segmented_);
	registration->setInputTarget (target_segmented_);
//	registration->setCorrespondenceEstimation(ce);
//	registration->setMaxCorrespondenceDistance(0.3);
	registration->setTransformationEpsilon (1e-8);
//	registration->setEuclideanFitnessEpsilon(0.01);
	registration->setMaximumIterations (30);
	registration->align(*source_registered_);
	transformation_matrix_ = registration->getFinalTransformation();
	fitness_score_ = registration->getFitnessScore();
	converged_ = registration->hasConverged();
}

inline Eigen::Matrix4f FeatureMatching1::getFinalTransformation() const {
	return transformation_matrix_;
}
//
//inline void FeatureMatching1::run() {
//	  visualizer_.spin();
//}
//
//inline void FeatureMatching1::keyboard_callback(
//		pcl::visualization::KeyboardEvent const & event, void* cookie) {
//	if (event.keyUp())
//	{
//	switch (event.getKeyCode())
//	{
//	  case '1':
//		if (!visualizer_.removePointCloud("source_points"))
//		{
//		  visualizer_.addPointCloud<PointRGBT>(source_, "source_points");
//		  visualizer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "source_points"); // BLUE
//
//		}
//		break;
//
//	  case '2':
//		if (!visualizer_.removePointCloud("target_points"))
//		{
//		  visualizer_.addPointCloud<PointRGBT>(target_, "target_points");
//		  visualizer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "target_points"); // RED
//
//		}
//		break;
//
//	  case '3':
//		if (!visualizer_.removePointCloud("source_segmented"))
//		{
//		  visualizer_.addPointCloud<PointRGBT>(source_segmented_, "source_segmented");
//		  visualizer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 255, "source_segmented"); // BLUE
//		}
//		break;
//
//	  case '4':
//		if (!visualizer_.removePointCloud("target_segmented"))
//		{
//		  visualizer_.addPointCloud<PointRGBT>(target_segmented_, "target_segmented");
//		  visualizer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 255, "target_segmented"); // BLUE
//
//		}
//		break;
//
//	  case '5':
//		  if (!visualizer_.removePointCloud("source_keypoints"))
//		  {
//			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> keypoint_color (source_keypoints_, 0, 0, 255);
//			//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> keypoint_color (source_keypoints_, "intensity");
//			visualizer_.addPointCloud(source_keypoints_, keypoint_color, "source_keypoints");
//		  }
//		  break;
//
//		case '6':
//		  if (!visualizer_.removePointCloud("target_keypoints"))
//		  {
//			//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> keypoint_color (target_keypoints_, "intensity");
//			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> keypoint_color (target_keypoints_, 255, 0, 0);
//			visualizer_.addPointCloud(target_keypoints_, keypoint_color, "target_keypoints");
//		  }
//		  break;
//
//		case '7':
//		        if (!show_source2target_)
//		          visualizer_.addCorrespondences<pcl::PointXYZI>(source_keypoints_, target_keypoints_, source2target_, "source2target");
//		        else
//		          visualizer_.removeCorrespondences("source2target");
//
//		        show_source2target_ = !show_source2target_;
//		        break;
//
//		      case '8':
//		        if (!show_target2source_)
//		          visualizer_.addCorrespondences<pcl::PointXYZI>(target_keypoints_, source_keypoints_, target2source_, "target2source");
//		        else
//		          visualizer_.removeCorrespondences("target2source");
//
//		        show_target2source_ = !show_target2source_;
//		break;
//
//		case '9':
//		if (!show_correspondences)
//		  visualizer_.addCorrespondences<pcl::PointXYZI>(source_keypoints_, target_keypoints_, *correspondences_, "correspondences");
//		else
//		  visualizer_.removeCorrespondences("correspondences");
//		show_correspondences = !show_correspondences;
//		break;
//
//	  case 'i':
//	  case 'I':
//		if (!visualizer_.removePointCloud("transformed")){
//		  visualizer_.addPointCloud<PointRGBT>(source_transformed_, "transformed");
//		  visualizer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 180, 255, "transformed"); // BLUE
//		}
//		break;
//
//	  case 'r':
//	  case 'R':
//		if (!visualizer_.removePointCloud("registered")){
//		  visualizer_.addPointCloud<PointRGBT>(source_registered_, "registered");
//		  visualizer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 255, 255, "registered"); // BLUE
//		}
//		break;
//	}
//	}
//}

//#include "FeatureMatching.hxx"
#endif /* SRC_FEATUREMATCHING1_H_ */
