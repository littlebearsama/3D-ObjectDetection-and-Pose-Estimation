#ifndef FAAT_PCL_REGISTRATION_VOXELBASED_CORRESPONDENCE_ESTIMATION_HPP_
#define FAAT_PCL_REGISTRATION_VOXELBASED_CORRESPONDENCE_ESTIMATION_HPP_

#include "voxel_based_correspondence_estimation.h"

template <typename PointSource, typename PointTarget, typename Scalar> void
faat_pcl::rec_3d_framework::VoxelBasedCorrespondenceEstimation<PointSource, PointTarget, Scalar>::determineCorrespondences (pcl::Correspondences &correspondences,
                                                                                                                                double /*max_distance*/) {
  if (!initCompute ())
    return;

  //find correspondences between input and target_ by looking into the distance transform thing...
  pcl::Correspondence corr;
  unsigned int nr_valid_correspondences = 0;

  //std::vector<int> indices = (*indices_);
  //std::random_shuffle(indices.begin(), indices.end());
  //indices.resize(std::min(static_cast<int>(indices.size()),100));
  correspondences.resize (indices_->size ());

  int idx_match;
  float distance;
  float color_distance = -1.f;

  for(size_t i=0; i < correspondences.size(); i++) {
    color_distance = -1.f;
    vgdt_target_->getCorrespondence(input_->points[(*indices_)[i]], &idx_match, &distance, sigma_, &color_distance);

    if(idx_match < 0)
      continue;

    if(distance > max_distance_)
      continue;

    if(sigma_ != -1.f && color_distance != -1.f) {
      if(color_distance < max_color_distance_)
      {
        //PCL_WARN("Rejected because color %f\n", color_distance, max_color_distance_);
        continue;
      }
    }

    /*corr.index_query = static_cast<int>(indices[i]);
    corr.index_match = idx_match;
    corr.distance = distance;//min_dist;*/
    correspondences[nr_valid_correspondences].index_query = static_cast<int>((*indices_)[i]);
    correspondences[nr_valid_correspondences].index_match = idx_match;
    correspondences[nr_valid_correspondences].distance = distance;
    nr_valid_correspondences++;
  }

  /*boost::shared_ptr<pcl::GeometricConsistencyGrouping<PointSource, PointTarget> >
          gcg_alg (new pcl::GeometricConsistencyGrouping<PointSource, PointTarget>);


  typename pcl::PointCloud<PointSource>::Ptr model_cloud(new pcl::PointCloud<PointSource>);
  vgdt_target_->getVoxelizedCloud(model_cloud);
  gcg_alg->setGCThreshold (10);
  gcg_alg->setGCSize (0.01);
  //gcg_alg->setSceneCloud (model_cloud);
  //gcg_alg->setInputCloud (input_);
  gcg_alg->setSceneCloud (input_);
  gcg_alg->setInputCloud (model_cloud);

  pcl::CorrespondencesPtr corr_ptr;
  corr_ptr.reset(new pcl::Correspondences(correspondences));
  gcg_alg->setModelSceneCorrespondences (corr_ptr);

  std::vector < pcl::Correspondences > corresp_clusters;
  gcg_alg->cluster (corresp_clusters);

  std::cout << "correspondence clusters:" << corresp_clusters.size() << std::endl;*/
  //std::cout << "Number of valid correspondences:" << indices_->size () << " " << nr_valid_correspondences << std::endl;

  //PCL_WARN("Number of valid correspondences... %d %d\n", indices_->size (), nr_valid_correspondences);
  correspondences.resize (nr_valid_correspondences);
  deinitCompute ();
}

#endif /* PCL_REGISTRATION_VOXELBASED_CORRESPONDENCE_ESTIMATION_HPP_ */
