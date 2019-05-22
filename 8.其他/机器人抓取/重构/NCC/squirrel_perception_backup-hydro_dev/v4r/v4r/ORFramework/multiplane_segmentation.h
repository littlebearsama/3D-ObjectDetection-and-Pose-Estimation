/*
 * multiplane_segmentation.h
 *
 *  Created on: Sep 25, 2013
 *      Author: aitor
 */

#ifndef FAAT_PCL_3D_REC_FRAMEWORK_MULTIPLANE_SEGMENTATION_H_
#define FAAT_PCL_3D_REC_FRAMEWORK_MULTIPLANE_SEGMENTATION_H_

#include "v4r/ORUtils/common_data_structures.h"

namespace faat_pcl
{
  template<typename PointT>
  class MultiPlaneSegmentation
  {
    private:
      typedef pcl::PointCloud<PointT> PointTCloud;
      typedef typename PointTCloud::Ptr PointTCloudPtr;
      typedef typename PointTCloud::ConstPtr PointTCloudConstPtr;
      PointTCloudPtr input_;
      int min_plane_inliers_;
      std::vector<PlaneModel<PointT> > models_;
      float resolution_;
      bool merge_planes_;
      pcl::PointCloud<pcl::Normal>::Ptr normal_cloud_;
      bool normals_set_;

    public:
      MultiPlaneSegmentation()
      {
        min_plane_inliers_ = 1000;
        resolution_ = 0.001f;
        merge_planes_ = false;
        normals_set_ = false;
      }

      void
      setMergePlanes(bool b)
      {
        merge_planes_ = b;
      }

      void setResolution(float f)
      {
        resolution_ = f;
      }

      void setMinPlaneInliers(int t)
      {
        min_plane_inliers_ = t;
      }

      void setInputCloud(PointTCloudPtr & input)
      {
        input_ = input;
      }

      void segment(bool force_unorganized=false);

      void setNormals(pcl::PointCloud<pcl::Normal>::Ptr & normal_cloud)
      {
          normal_cloud_ = normal_cloud;
          normals_set_ = true;
      }

      std::vector<PlaneModel<PointT> > getModels()
      {
        return models_;
      }
  };
}

#endif /* FAAT_PCL_3D_REC_FRAMEWORK_MULTIPLANE_SEGMENTATION_H_ */
