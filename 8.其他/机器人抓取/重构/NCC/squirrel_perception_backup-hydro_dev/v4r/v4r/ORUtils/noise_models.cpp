/*
 * noise_models.cpp
 *
 *  Created on: Oct 28, 2013
 *      Author: aitor
 */

#include <pcl/common/angles.h>
#include <v4r/OREdgeDetector/organized_edge_detection.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "noise_models.h"
#include "organized_edge_detection.h"

template<typename PointT>
faat_pcl::utils::noise_models::NguyenNoiseModel<PointT>::NguyenNoiseModel ()
{
  max_angle_ = 70.f;
  lateral_sigma_ = 0.002f;
}

template<typename PointT>
void
faat_pcl::utils::noise_models::NguyenNoiseModel<PointT>::compute ()
{
  weights_.clear();
  weights_.resize(input_->points.size(), 1.f);
  discontinuity_edges_.indices.clear();

  //compute depth discontinuity edges
  faat_pcl::OrganizedEdgeBase<PointT, pcl::Label> oed;
  oed.setDepthDisconThreshold (0.05f); //at 1m, adapted linearly with depth
  oed.setMaxSearchNeighbors(100);
  oed.setEdgeType (faat_pcl::OrganizedEdgeBase<PointT, pcl::Label>::EDGELABEL_OCCLUDING
  | faat_pcl::OrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label>::EDGELABEL_OCCLUDED
  | faat_pcl::OrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label>::EDGELABEL_NAN_BOUNDARY
  );
  oed.setInputCloud (input_);

  pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
  std::vector<pcl::PointIndices> indices2;
  oed.compute (*labels, indices2);

  for (size_t j = 0; j < indices2.size (); j++)
  {
    for (size_t i = 0; i < indices2[j].indices.size (); i++)
    {
      discontinuity_edges_.indices.push_back(indices2[j].indices[i]);
    }
  }

  for(size_t i=0; i < input_->points.size(); i++)
  {
    const Eigen::Vector3f & np = normals_->points[i].getNormalVector3fMap();

    if(!pcl_isfinite(input_->points[i].z) || !pcl_isfinite(np[2]))
    {
      weights_[i] = 0;
      continue;
    }

    //origin to pint
    //Eigen::Vector3f o2p = input_->points[i].getVector3fMap() * -1.f;
    Eigen::Vector3f o2p = Eigen::Vector3f::UnitZ() * -1.f;

    o2p.normalize();
    float angle = pcl::rad2deg(acos(o2p.dot(np)));
    if(angle > max_angle_)
    {
      weights_[i] = 1.f - (angle - max_angle_) / (90.f - max_angle_);
    }
    else
    {
      //weights_[i] = 1.f - 0.2f * ((std::max(angle, 30.f) - 30.f) / (max_angle_ - 30.f));
    }

    //std::cout << angle << " " << weights_[i] << std::endl;
    //weights_[i] = 1.f - ( angle )
  }

  //dilate edge pixels checking that the distance is ok and keeping always the minimum euclidean distance for each dilated pixel
  if (use_depth_edges_)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    edge_cloud->width = input_->width;
    edge_cloud->height = input_->height;
    edge_cloud->points.resize (input_->points.size ());

    Eigen::Vector3f nan3f (std::numeric_limits<float>::quiet_NaN (), std::numeric_limits<float>::quiet_NaN (),
                           std::numeric_limits<float>::quiet_NaN ());

    for (size_t i = 0; i < edge_cloud->points.size (); i++)
      edge_cloud->points[i].getVector3fMap () = nan3f;

    for (size_t i = 0; i < discontinuity_edges_.indices.size (); i++)
      edge_cloud->points[discontinuity_edges_.indices[i]].getVector3fMap () = input_->points[discontinuity_edges_.indices[i]].getVector3fMap ();

    int wsize = 5;
    int wsize2 = wsize / 2;
    int dilate_iterations = 3;

    /*pcl::visualization::PCLVisualizer vis("DILATION");
     vis.addPointCloud(edge_cloud);
     vis.spin();*/

    std::vector<float> dist_to_edge(input_->points.size(), std::numeric_limits<float>::infinity());
    for (size_t i = 0; i < discontinuity_edges_.indices.size (); i++)
      dist_to_edge[discontinuity_edges_.indices[i]] = 0.f;

    for (int i = 0; i < dilate_iterations; i++)
    {

      pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud2 (new pcl::PointCloud<pcl::PointXYZ> (*edge_cloud));

      for (int v = wsize2; v < static_cast<int>(input_->width - wsize2); v++)
      {
        for (int u = wsize2; u < static_cast<int>(input_->height - wsize2); u++)
        {
          if (!pcl_isfinite(edge_cloud->at(v,u).z))
            continue;

          //try growing edge cloud, checking that distance is smaller than 3*lateral_sigma
          for (int kv = (v - wsize2); kv <= (v + wsize2); kv++)
          {
            for (int ku = (u - wsize2); ku <= (u + wsize2); ku++)
            {
              if (pcl_isfinite(edge_cloud->at(kv,ku).z)) //already edge pixel
                continue;

              if (!pcl_isfinite(input_->at(kv,ku).z))
                continue;

              if (!pcl_isfinite(input_->at(v,u).z))
                continue;

              //std::cout << u << " " << v << " " << ku << " " << kv << std::endl;
              int idx_u_v = u * input_->width + v;
              float dist = dist_to_edge[idx_u_v] + (input_->at (v, u).getVector3fMap () - input_->at (kv, ku).getVector3fMap ()).norm ();
              //std::cout << dist << " " << lateral_sigma_ * 3.f << std::endl;
              if (dist < (lateral_sigma_ * 3.f))
              {
                //grow
                edge_cloud2->at (kv, ku).getVector3fMap () = input_->at (kv, ku).getVector3fMap ();

                int idx = ku * input_->width + kv;
                dist_to_edge[idx] = std::min(dist_to_edge[idx], dist);
              }
            }
          }
        }
      }

      edge_cloud = edge_cloud2;
      /*vis.removeAllPointClouds();
       vis.addPointCloud(edge_cloud);
       vis.spin();*/
    }

    for (int i = 0; i < weights_.size (); i++)
    {
      int v, u;
      v = i / input_->width;
      u = i % input_->width;
      if (!pcl_isfinite(edge_cloud->at(u,v).z))
        continue;

      assert(pcl_isfinite(dist_to_edge[i]));
      //adapt weight
      weights_[i] *= 1.f - 0.5f * std::exp((dist_to_edge[i] * dist_to_edge[i]) / (lateral_sigma_ * lateral_sigma_));
    }
  }

  for(size_t i=0; i < input_->points.size(); i++)
  {
      if(weights_[i] < 0.f)
      {
          weights_[i] = 0.f;
      }
  }
}

/*template<typename PointT>
void
faat_pcl::utils::noise_models::NguyenNoiseModel<PointT>::getFilteredCloud(PointTPtr & filtered, float w_t)
{
  Eigen::Vector3f nan3f(std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::quiet_NaN());
  filtered.reset(new pcl::PointCloud<PointT>(*input_));
  for(size_t i=0; i < input_->points.size(); i++)
  {
    if(weights_[i] < w_t)
    {
      //filtered->points[i].getVector3fMap() = nan3f;
      filtered->points[i].r = 255;
      filtered->points[i].g = 0;
      filtered->points[i].b = 0;
    }

    if(!pcl_isfinite( input_->points[i].z))
    {
      filtered->points[i].r = 255;
      filtered->points[i].g = 255;
      filtered->points[i].b = 0;
    }
  }
}*/

template<typename PointT>
void
faat_pcl::utils::noise_models::NguyenNoiseModel<PointT>::getFilteredCloudRemovingPoints(PointTPtr & filtered, float w_t)
{
  Eigen::Vector3f nan3f(std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::quiet_NaN());

  filtered.reset(new pcl::PointCloud<PointT>(*input_));
  for(size_t i=0; i < input_->points.size(); i++)
  {
    if(weights_[i] < w_t)
    {
      filtered->points[i].x = std::numeric_limits<float>::quiet_NaN();
      filtered->points[i].y = std::numeric_limits<float>::quiet_NaN();
      filtered->points[i].z = std::numeric_limits<float>::quiet_NaN();
    }
  }
}

template<typename PointT>
void
faat_pcl::utils::noise_models::NguyenNoiseModel<PointT>:: getFilteredCloudRemovingPoints(PointTPtr & filtered, float w_t, std::vector<int> & kept)
{
    Eigen::Vector3f nan3f(std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::quiet_NaN());

    filtered.reset(new pcl::PointCloud<PointT>(*input_));
    for(size_t i=0; i < input_->points.size(); i++)
    {
      if(weights_[i] < w_t)
      {
        filtered->points[i].getVector3fMap() = nan3f;
      }
      else
      {
          kept.push_back(i);
      }
    }
}

template class faat_pcl::utils::noise_models::NguyenNoiseModel<pcl::PointXYZRGB>;
template class faat_pcl::utils::noise_models::NguyenNoiseModel<pcl::PointXYZ>;
