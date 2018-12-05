template<typename FeatureType>
FeatureMatching<FeatureType>::FeatureMatching(boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGBA, pcl::PointXYZI> >keypoint_detector,
                                        typename pcl::Feature<pcl::PointXYZRGBA, FeatureType>::Ptr feature_extractor,
                                        boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstructor,
                                        typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source,
                                        typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr target)
: source_keypoints_ (new pcl::PointCloud<pcl::PointXYZI> ())
, target_keypoints_ (new pcl::PointCloud<pcl::PointXYZI> ())
, keypoint_detector_ (keypoint_detector)
, feature_extractor_ (feature_extractor)
, surface_reconstructor_ (surface_reconstructor)
, source_ (source)
, target_ (target)
, source_segmented_ (new pcl::PointCloud<pcl::PointXYZRGBA>)
, target_segmented_ (new pcl::PointCloud<pcl::PointXYZRGBA>)
, source_transformed_ (new pcl::PointCloud<pcl::PointXYZRGBA>)
, source_registered_ (new pcl::PointCloud<pcl::PointXYZRGBA>)
, source_features_ (new pcl::PointCloud<FeatureType>)
, target_features_ (new pcl::PointCloud<FeatureType>)
, correspondences_ (new pcl::Correspondences)
, show_source2target_ (false)
, show_target2source_ (false)
, show_correspondences (false)
, fitness_score_(0)
{ }

template<typename FeatureType>
void FeatureMatching<FeatureType>::match() {
	//  visualizer_.registerKeyboardCallback(&FeatureMatching::keyboard_callback, *this, 0);

	  segmentation (source_, source_segmented_);
	  segmentation (target_, target_segmented_);

	  detectKeypoints (source_segmented_, source_keypoints_);
	  detectKeypoints (target_segmented_, target_keypoints_);

	  if(source_keypoints_->points.size() == 0 || target_keypoints_->points.size() == 0  ){
		  correspondences_->resize(0);
		  fitness_score_ = 1e8;
		  return;
	  }

	  extractDescriptors (source_segmented_, source_keypoints_, source_features_);
	  extractDescriptors (target_segmented_, target_keypoints_, target_features_);

	  findCorrespondences (source_features_, target_features_, source2target_);
	  findCorrespondences (target_features_, source_features_, target2source_);

	  filterCorrespondences ();

	  determineInitialTransformation ();
	  determineFinalTransformation ();

	//  reconstructSurface ();
}

template<typename FeatureType>
void FeatureMatching<FeatureType>::segmentation (typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source, typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmented) const
{
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
  //	seg.setMaxIterations(1000);
  seg.setEpsAngle(pcl::deg2rad(5.0));
  seg.setDistanceThreshold(0.01);

  seg.setInputCloud (source);
  seg.segment (*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
  if(inliers->indices.size() > 0 ){
	  extract.setInputCloud (source);
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
	  pcl::removeNaNFromPointCloud<PointRGBT>(*source, *segmented, indices);
	  // cout << "OK" << endl;
  }
  // cout << "clustering..." << std::flush;
  // euclidean clustering
  typename pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
  tree->setInputCloud (segmented);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> clustering;
  clustering.setClusterTolerance (0.5); // 50cm
  clustering.setMinClusterSize (1000);
  clustering.setMaxClusterSize (25000);
  clustering.setSearchMethod (tree);
  clustering.setInputCloud(segmented);
  clustering.extract (cluster_indices);

  if (cluster_indices.size() > 0)//use largest cluster
  {
    // cout << cluster_indices.size() << " clusters found";
    if (cluster_indices.size() > 1)
//       cout <<" Using largest one...";
     cout << endl;

    typename pcl::IndicesPtr indices (new std::vector<int>);
    *indices = cluster_indices[0].indices;
    extract.setInputCloud (segmented);
    extract.setIndices (indices);
    extract.setNegative (false);

    extract.filter (*segmented);
  }
}

template<typename FeatureType>
void FeatureMatching<FeatureType>::detectKeypoints (typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints) const
{
  // // cout << "keypoint detection..." << std::flush;
  keypoint_detector_->setInputCloud(input);
  keypoint_detector_->compute(*keypoints);
  // cout << "OK. keypoints found: " << keypoints->points.size() << endl;
}

template<typename FeatureType>
void FeatureMatching<FeatureType>::extractDescriptors (typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr input, typename pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, typename pcl::PointCloud<FeatureType>::Ptr features)
{
  typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr kpts(new pcl::PointCloud<pcl::PointXYZRGBA>);
  kpts->points.resize(keypoints->points.size());

  pcl::copyPointCloud(*keypoints, *kpts);

  typename pcl::FeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, FeatureType>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, FeatureType> > (feature_extractor_);

  feature_extractor_->setSearchSurface(input);
  feature_extractor_->setInputCloud(kpts);

  if (feature_from_normals)
  //if (boost::dynamic_pointer_cast<typename pcl::FeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, FeatureType> > (feature_extractor_))
  {
    // cout << "normal estimation..." << std::flush;
    typename pcl::PointCloud<pcl::Normal>::Ptr normals (new  pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimation;
    normal_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZRGBA>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGBA>));
    normal_estimation.setRadiusSearch (0.1);
    normal_estimation.setInputCloud (input);
    normal_estimation.compute (*normals);
    feature_from_normals->setInputNormals(normals);
    // cout << "OK" << endl;
  }

  // cout << "descriptor extraction..." << std::flush;
  feature_extractor_->compute (*features);
  // cout << "OK" << endl;
}

template<typename FeatureType>
void FeatureMatching<FeatureType>::findCorrespondences (typename pcl::PointCloud<FeatureType>::Ptr source, typename pcl::PointCloud<FeatureType>::Ptr target, std::vector<int>& correspondences) const
{
  // cout << "correspondence assignment..." << std::flush;
  correspondences.resize (source->size());

  // Use a KdTree to search for the nearest matches in feature space
  pcl::KdTreeFLANN<FeatureType> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target);

  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (int i = 0; i < static_cast<int> (source->size ()); ++i)
  {
    descriptor_kdtree.nearestKSearch (*source, i, k, k_indices, k_squared_distances);
    correspondences[i] = k_indices[0];
  }
  // cout << "OK" << endl;
}

template<typename FeatureType>
void FeatureMatching<FeatureType>::filterCorrespondences ()
{
  // cout << "correspondence rejection..." << std::flush;
  std::vector<std::pair<unsigned, unsigned> > correspondences;
  for (unsigned cIdx = 0; cIdx < source2target_.size (); ++cIdx)
    if (target2source_[source2target_[cIdx]] == static_cast<int> (cIdx))
      correspondences.push_back(std::make_pair(cIdx, source2target_[cIdx]));

  correspondences_->resize (correspondences.size());
  for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx)
  {
    (*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
    (*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
  }

  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector;
  rejector.setInputSource (source_keypoints_);
  rejector.setInputTarget (target_keypoints_);
  rejector.setInputCorrespondences(correspondences_);
  rejector.getCorrespondences(*correspondences_);
  // cout << "OK" << endl;
}

template<typename FeatureType>
void FeatureMatching<FeatureType>::determineInitialTransformation ()
{
  // cout << "initial alignment..." << std::flush;
  pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Ptr transformation_estimation (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI>);

  transformation_estimation->estimateRigidTransformation (*source_keypoints_, *target_keypoints_, *correspondences_, initial_transformation_matrix_);

  pcl::transformPointCloud(*source_segmented_, *source_transformed_, initial_transformation_matrix_);
  // cout << "OK" << endl;
}

template<typename FeatureType>
void FeatureMatching<FeatureType>::determineFinalTransformation ()
{
  // cout << "final registration..." << std::flush;
  pcl::Registration<pcl::PointXYZRGBA, pcl::PointXYZRGBA>::Ptr registration (new pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>);
  registration->setInputSource (source_transformed_);
  //registration->setInputSource (source_segmented_);
  registration->setInputTarget (target_segmented_);
  registration->setMaxCorrespondenceDistance(0.4);
  registration->setRANSACOutlierRejectionThreshold (0.4);
  registration->setTransformationEpsilon (0.001);
  registration->setMaximumIterations (50);
  registration->align(*source_registered_);
  transformation_matrix_ = registration->getFinalTransformation();
  fitness_score_ = registration->getFitnessScore();
  // cout << "OK" << endl;
}

template<typename FeatureType>
void FeatureMatching<FeatureType>::reconstructSurface ()
{
  // cout << "surface reconstruction..." << std::flush;
  // merge the transformed and the target point cloud
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr merged (new pcl::PointCloud<pcl::PointXYZRGBA>);
  *merged = *source_registered_;
  *merged += *target_segmented_;

  // apply grid filtering to reduce amount of points as well as to make them uniform distributed
  pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;
  voxel_grid.setInputCloud(merged);
  voxel_grid.setLeafSize (0.002f, 0.002f, 0.002f);
  voxel_grid.setDownsampleAllData(true);
  voxel_grid.filter(*merged);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr vertices (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::copyPointCloud(*merged, *vertices);

  pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal> normal_estimation;
  normal_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZRGBA>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGBA>));
  normal_estimation.setRadiusSearch (0.01);
  normal_estimation.setInputCloud (merged);
  normal_estimation.compute (*vertices);

  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree->setInputCloud (vertices);

  surface_reconstructor_->setSearchMethod(tree);
  surface_reconstructor_->setInputCloud(vertices);
  surface_reconstructor_->reconstruct(surface_);
  // cout << "OK" << endl;
}

//template<typename FeatureType>
//void FeatureMatching<FeatureType>::run()
//{
//  visualizer_.spin ();
//}

template<typename FeatureType>
inline int FeatureMatching<FeatureType>::getCorrespondencesSize() {
	return correspondences_->size();
}

template<typename FeatureType>
inline double FeatureMatching<FeatureType>::getFitnessScore() {
	return fitness_score_;
}
//template<typename FeatureType>
//void FeatureMatching<FeatureType>::keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
//{
//  if (event.keyUp())
//  {
//    switch (event.getKeyCode())
//    {
//      case '1':
//        if (!visualizer_.removePointCloud("source_points"))
//        {
//          visualizer_.addPointCloud(source_, "source_points");
//        }
//        break;
//
//      case '2':
//        if (!visualizer_.removePointCloud("target_points"))
//        {
//          visualizer_.addPointCloud(target_, "target_points");
//        }
//        break;
//
//      case '3':
//        if (!visualizer_.removePointCloud("source_segmented"))
//        {
//          visualizer_.addPointCloud(source_segmented_, "source_segmented");
//        }
//        break;
//
//      case '4':
//        if (!visualizer_.removePointCloud("target_segmented"))
//        {
//          visualizer_.addPointCloud(target_segmented_, "target_segmented");
//        }
//        break;
//
//      case '5':
//        if (!visualizer_.removePointCloud("source_keypoints"))
//        {
//          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> keypoint_color (source_keypoints_, 0, 0, 255);
//          //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> keypoint_color (source_keypoints_, "intensity");
//          visualizer_.addPointCloud(source_keypoints_, keypoint_color, "source_keypoints");
//        }
//        break;
//
//      case '6':
//        if (!visualizer_.removePointCloud("target_keypoints"))
//        {
//          //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> keypoint_color (target_keypoints_, "intensity");
//          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> keypoint_color (target_keypoints_, 255, 0, 0);
//          visualizer_.addPointCloud(target_keypoints_, keypoint_color, "target_keypoints");
//        }
//        break;
//
//      case '7':
//        if (!show_source2target_)
//          visualizer_.addCorrespondences<pcl::PointXYZI>(source_keypoints_, target_keypoints_, source2target_, "source2target");
//        else
//          visualizer_.removeCorrespondences("source2target");
//
//        show_source2target_ = !show_source2target_;
//        break;
//
//      case '8':
//        if (!show_target2source_)
//          visualizer_.addCorrespondences<pcl::PointXYZI>(target_keypoints_, source_keypoints_, target2source_, "target2source");
//        else
//          visualizer_.removeCorrespondences("target2source");
//
//        show_target2source_ = !show_target2source_;
//        break;
//
//      case '9':
//        if (!show_correspondences)
//          visualizer_.addCorrespondences<pcl::PointXYZI>(source_keypoints_, target_keypoints_, *correspondences_, "correspondences");
//        else
//          visualizer_.removeCorrespondences("correspondences");
//        show_correspondences = !show_correspondences;
//        break;
//
//      case 'i':
//      case 'I':
//        if (!visualizer_.removePointCloud("transformed"))
//          visualizer_.addPointCloud(source_transformed_, "transformed");
//        break;
//
//      case 'r':
//      case 'R':
//        if (!visualizer_.removePointCloud("registered"))
//          visualizer_.addPointCloud(source_registered_, "registered");
//        break;
//
//      case 't':
//      case 'T':
//          visualizer_.addPolygonMesh(surface_, "surface");
//        break;
//    }
//  }
//}
