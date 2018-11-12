/*
 * FeatureMatching.h
 *
 *  Created on: Oct 29, 2018
 *      Author: xenakis
 */

#ifndef SRC_FEATUREMATCHING_H_
#define SRC_FEATUREMATCHING_H_

#include <amcl_depth_types.h>

#include <vector>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/angles.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
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
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>



template<typename FeatureType>
class FeatureMatching
{
  public:
    FeatureMatching (boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGBA, pcl::PointXYZI> > keypoint_detector,
                  typename pcl::Feature<pcl::PointXYZRGBA, FeatureType>::Ptr feature_extractor,
                  boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstructor,
                  typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source,
                  typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr target);

    void match();
    /**
     * @brief starts the event loop for the visualizer
     */
//    void run ();

    int getCorrespondencesSize();

    double getFitnessScore();
//    pcl::visualization::PCLVisualizer visualizer_;

  protected:
    /**
     * @brief remove plane and select largest cluster as input object
     * @param input the input point cloud
     * @param segmented the resulting segmented point cloud containing only points of the largest cluster
     */
    void segmentation (typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr input, typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmented) const;

    /**
     * @brief Detects key points in the input point cloud
     * @param input the input point cloud
     * @param keypoints the resulting key points. Note that they are not necessarily a subset of the input cloud
     */
    void detectKeypoints (typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints) const;

    /**
     * @brief extract descriptors for given key points
     * @param input point cloud to be used for descriptor extraction
     * @param keypoints locations where descriptors are to be extracted
     * @param features resulting descriptors
     */
    void extractDescriptors (typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr input, typename pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, typename pcl::PointCloud<FeatureType>::Ptr features);

    /**
     * @brief find corresponding features based on some metric
     * @param source source feature descriptors
     * @param target target feature descriptors
     * @param correspondences indices out of the target descriptors that correspond (nearest neighbor) to the source descriptors
     */
    void findCorrespondences (typename pcl::PointCloud<FeatureType>::Ptr source, typename pcl::PointCloud<FeatureType>::Ptr target, std::vector<int>& correspondences) const;

    /**
     * @brief  remove non-consistent correspondences
     */
    void filterCorrespondences ();

    /**
     * @brief calculate the initial rigid transformation from filtered corresponding keypoints
     */
    void determineInitialTransformation ();

    /**
     * @brief calculate the final rigid transformation using ICP over all points
     */
    void determineFinalTransformation ();

    /**
     * @brief reconstructs the surface from merged point clouds
     */
    void reconstructSurface ();

    /**
     * @brief callback to handle keyboard events
     * @param event object containing information about the event. e.g. type (press, release) etc.
     * @param cookie user defined data passed during registration of the callback
     */
//    void keyboard_callback (const pcl::visualization::KeyboardEvent& event, void* cookie);


  private:
//    pcl::visualization::PCLVisualizer visualizer_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_;
    boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGBA, pcl::PointXYZI> > keypoint_detector_;
    typename pcl::Feature<pcl::PointXYZRGBA, FeatureType>::Ptr feature_extractor_;
    boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstructor_;
    typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source_;
    typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr target_;
    typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_segmented_;
    typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_segmented_;
    typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_transformed_;
    typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_registered_;
    typename pcl::PolygonMesh surface_;
    typename pcl::PointCloud<FeatureType>::Ptr source_features_;
    typename pcl::PointCloud<FeatureType>::Ptr target_features_;
    std::vector<int> source2target_;
    std::vector<int> target2source_;
    pcl::CorrespondencesPtr correspondences_;
    Eigen::Matrix4f initial_transformation_matrix_;
    Eigen::Matrix4f transformation_matrix_;
    double fitness_score_;
    bool show_source2target_;
    bool show_target2source_;
    bool show_correspondences;
};

#include "FeatureMatching.hxx"
#endif /* SRC_FEATUREMATCHING_H_ */
