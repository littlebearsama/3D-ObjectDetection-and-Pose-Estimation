/*
 * registration_utils.h
 *
 *  Created on: Oct 17, 2013
 *      Author: aitor
 */

#ifndef REGISTRATION_UTILS_H_
#define REGISTRATION_UTILS_H_

#include <pcl/octree/octree.h>
#include <pcl/features/organized_edge_detection.h>

namespace registration_utils
{

  template<class PointT>
  inline void getRGBEdges(typename pcl::PointCloud<PointT>::Ptr & cloud,
                                       std::vector<int> & indices,
                                       float low = 150.f, float high = 200.f,
                                       float max_dist = std::numeric_limits<float>::max())
  {
    typedef typename pcl::PointCloud<PointT> PointCloudT;
    typedef typename PointCloudT::Ptr PointCloudTPtr;
    //COMPUTE edges close to the boundaries
    pcl::OrganizedEdgeFromRGB<PointT, pcl::Label> oed;
    oed.setDepthDisconThreshold (0.03f);
    oed.setRGBCannyLowThreshold (low);
    oed.setRGBCannyHighThreshold (high);
    oed.setEdgeType (pcl::OrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label>::EDGELABEL_RGB_CANNY);
    oed.setInputCloud (cloud);
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> indices2;
    oed.compute (*labels, indices2);

    for (size_t j = 0; j < indices2.size (); j++)
    {
      if(indices2[j].indices.size () > 0)
      {
        for(size_t k=0; k < indices2[j].indices.size(); k++)
        {
            if(cloud->points[indices2[j].indices[k]].z > max_dist)
                continue;

            if(!pcl_isfinite(cloud->points[indices2[j].indices[k]].z))
                continue;

            indices.push_back(indices2[j].indices[k]);
        }
      }
    }

  }

  template<class PointT>
  inline void getIndicesCloseToBoundaries(typename pcl::PointCloud<PointT>::Ptr & cloud,
                                       std::vector<int> & indices,
                                       float thres=0.001f)
  {
    typedef typename pcl::PointCloud<PointT> PointCloudT;
    typedef typename PointCloudT::Ptr PointCloudTPtr;
    //COMPUTE edges close to the boundaries
    pcl::OrganizedEdgeFromRGB<PointT, pcl::Label> oed;
    oed.setDepthDisconThreshold (0.03f);
    oed.setEdgeType (pcl::OrganizedEdgeBase<PointT, pcl::Label>::EDGELABEL_OCCLUDING);
    oed.setInputCloud (cloud);
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> indices2;
    oed.compute (*labels, indices2);
    PointCloudTPtr edges (new PointCloudT);
    for (size_t j = 0; j < indices2.size (); j++)
    {
      for (size_t i = 0; i < indices2[j].indices.size (); i++)
      {
        PointT pl;
        pl = cloud->points[indices2[j].indices[i]];
        edges->push_back (pl);
      }
    }

    //filter points on the cloud that are too close to the boundary edges
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    pcl::octree::OctreePointCloudSearch<PointT> octree (0.003);
    octree.setInputCloud (edges);
    octree.addPointsFromInputCloud ();

    std::vector<int> close_to_boundary;
    for (size_t k = 0; k < cloud->points.size (); k++)
    {
      if (!pcl_isfinite(cloud->points[k].z))
        continue;

      if (octree.nearestKSearch (cloud->points[k], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        float d = sqrt (pointNKNSquaredDistance[0]);
        if (d < thres)
          close_to_boundary.push_back (static_cast<int> (k));
      }
    }

    indices = close_to_boundary;
  }

  inline std::vector<bool> mask_and(std::vector<bool> & mask1, std::vector<bool> & mask2)
  {
    assert(mask1.size() == mask2.size());
    std::vector<bool> mask(mask1.size(), false);
    for(size_t i=0; i < mask1.size(); i++)
    {
      if(mask1[i] && mask2[i]) mask[i] = true;
    }
    return mask;
  }

  inline std::vector<bool> indicesToMask(std::vector<int> & indices, int size, bool negative=false)
  {
    std::vector<bool> mask;
    mask.resize(size, negative);
    for(size_t i=0; i < indices.size(); i++)
      mask[indices[i]] = !negative;

    return mask;
  }

  inline std::vector<int> maskToIndices(std::vector<bool> & mask)
  {
    std::vector<int> indices;
    for(size_t i=0; i < mask.size(); i++)
    {
      if(mask[i])
        indices.push_back(i);
    }

    return indices;
  }
}

#endif /* REGISTRATION_UTILS_H_ */
