/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/instantiate.hpp>
#include "hv_go_3D.h"
#include "occlusion_reasoning.hpp"
#include <pcl/features/normal_3d_omp.h>
#include <functional>
#include <numeric>

template<typename ModelT, typename SceneT>
bool
faat_pcl::GO3D<ModelT, SceneT>::getInlierOutliersCloud(int hyp_idx, typename pcl::PointCloud<ModelT>::Ptr & cloud)
{
  if(hyp_idx < 0 || hyp_idx > (recognition_models_.size() - 1))
    return false;

  boost::shared_ptr<GHVRecognitionModel<ModelT> > recog_model = recognition_models_[hyp_idx];
  cloud.reset(new pcl::PointCloud<ModelT>(*recog_model->cloud_));


  for(size_t i=0; i < cloud->points.size(); i++)
  {
    cloud->points[i].r = 0;
    cloud->points[i].g = 255;
    cloud->points[i].b = 0;
  }

  for(size_t i=0; i < recog_model->outlier_indices_.size(); i++)
  {
    cloud->points[recog_model->outlier_indices_[i]].r = 255;
    cloud->points[recog_model->outlier_indices_[i]].g = 0;
    cloud->points[recog_model->outlier_indices_[i]].b = 0;
  }

  return true;
}

template<typename ModelT, typename SceneT>
void
faat_pcl::GO3D<ModelT, SceneT>::addModels (std::vector<typename pcl::PointCloud<ModelT>::ConstPtr> & models, bool occlusion_reasoning)
{
  std::cout << "Called GO3D addModels" << std::endl;
  mask_.clear();

  if (!occlusion_reasoning)
    visible_models_ = models;
  else
  {
    visible_indices_.resize(models.size());
    //pcl::visualization::PCLVisualizer vis("visible model");
    for (size_t i = 0; i < models.size (); i++)
    {

      //a point is occluded if it is occluded in all views
      typename pcl::PointCloud<ModelT>::Ptr filtered (new pcl::PointCloud<ModelT> ());

      //scene-occlusions
      for(size_t k=0; k < occ_clouds_.size(); k++)
      {
        //transform model to camera coordinate
        typename pcl::PointCloud<ModelT>::Ptr model_in_view_coordinates(new pcl::PointCloud<ModelT> ());
        Eigen::Matrix4f trans =  absolute_poses_camera_to_global_[k].inverse();
        pcl::transformPointCloud(*models[i], *model_in_view_coordinates, trans);
        typename pcl::PointCloud<ModelT>::ConstPtr const_filtered(new pcl::PointCloud<ModelT> (*model_in_view_coordinates));

        std::vector<int> indices_cloud_occlusion;
        filtered = faat_pcl::occlusion_reasoning::filter<ModelT,SceneT> (occ_clouds_[k], const_filtered, 525.f, occlusion_thres_, indices_cloud_occlusion);

        std::vector<int> final_indices = indices_cloud_occlusion;
        final_indices.resize(indices_cloud_occlusion.size());

        visible_indices_[i].insert(visible_indices_[i].end(), final_indices.begin(), final_indices.end());
      }

      std::set<int> s( visible_indices_[i].begin(), visible_indices_[i].end() );
      visible_indices_[i].assign( s.begin(), s.end() );

      pcl::copyPointCloud(*models[i], visible_indices_[i], *filtered);

      if(normals_set_ && requires_normals_) {
        pcl::PointCloud<pcl::Normal>::Ptr filtered_normals (new pcl::PointCloud<pcl::Normal> ());
        pcl::copyPointCloud(*complete_normal_models_[i], visible_indices_[i], *filtered_normals);
        visible_normal_models_.push_back(filtered_normals);
      }

      /*pcl::visualization::PointCloudColorHandlerRGBField<ModelT> handler (filtered);
      vis.addPointCloud(filtered, handler, "model");
      vis.spin();
      vis.removeAllPointClouds();*/

      visible_models_.push_back (filtered);
    }

    complete_models_ = models;
  }

  normals_set_ = false;
}

template<typename ModelT, typename SceneT> float faat_pcl::GO3D<ModelT, SceneT>::sRGB_LUT[256] = {- 1};
template<typename ModelT, typename SceneT> float faat_pcl::GO3D<ModelT, SceneT>::sXYZ_LUT[4000] = {- 1};

//template class FAAT_REC_API faat_pcl::GO3D<pcl::PointXYZ,pcl::PointXYZ>;
template class FAAT_REC_API faat_pcl::GO3D<pcl::PointXYZRGB,pcl::PointXYZRGB>;
