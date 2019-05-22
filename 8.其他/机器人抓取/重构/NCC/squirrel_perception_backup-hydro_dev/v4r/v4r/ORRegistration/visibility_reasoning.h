/*
 * visibility_reasoning.h
 *
 *  Created on: Mar 19, 2013
 *      Author: aitor
 */

#ifndef FAAT_PCL_REGISTRATION_VISIBILITY_REASONING_H_
#define FAAT_PCL_REGISTRATION_VISIBILITY_REASONING_H_

#include "pcl/common/common.h"

namespace faat_pcl
{
  namespace registration
  {
    template<typename PointT>
    class VisibilityReasoning
    {
        typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
      float focal_length_; float cx_; float cy_;
      float tss_;
      int fsv_used_;
      public:
        VisibilityReasoning(float fc, float cx, float cy)
        {
          focal_length_ = fc;
          cx_ = cx;
          cy_ = cy;
          tss_ = 0.01f;
          fsv_used_ = 0;
        }

        int getFSVUsedPoints()
        {
          return fsv_used_;
        }

        int
        computeRangeDifferencesWhereObserved(const typename pcl::PointCloud<PointT>::ConstPtr & im1, const typename pcl::PointCloud<PointT>::ConstPtr & im2, std::vector<float> & range_diff);

        int
        computeRangeDifferencesWhereObservedWithIndicesBack(const typename pcl::PointCloud<PointT>::ConstPtr & im1, const typename pcl::PointCloud<PointT>::ConstPtr & im2, std::vector<float> & range_diff, std::vector<int> & indices);

        float computeFSV(const typename pcl::PointCloud<PointT>::ConstPtr &im1,
                           const typename pcl::PointCloud<PointT>::ConstPtr &im2,
                           Eigen::Matrix4f pose_2_to_1 = Eigen::Matrix4f::Identity());

        float computeFSVWithNormals(const typename pcl::PointCloud<PointT>::ConstPtr &im1,
                                    const typename pcl::PointCloud<PointT>::ConstPtr &im2,
                                    pcl::PointCloud<pcl::Normal>::Ptr & normals);

        float computeOSV(const typename pcl::PointCloud<PointT>::ConstPtr &im1,
                           const typename pcl::PointCloud<PointT>::ConstPtr &im2,
                           Eigen::Matrix4f pose_2_to_1 = Eigen::Matrix4f::Identity());

        void setThresholdTSS(float t)
        {
          tss_ = t;
        }

        float computeFocalLength(int cx, int cy, const typename pcl::PointCloud<PointT>::ConstPtr & cloud);

        void computeRangeImage(int cx, int cy, float fl, const typename pcl::PointCloud<PointT>::ConstPtr & cloud, typename pcl::PointCloud<PointT>::Ptr & range_image);
    };
  }
}

#endif /* VISIBILITY_REASONING_H_ */
