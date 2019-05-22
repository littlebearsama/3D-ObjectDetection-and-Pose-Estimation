/*
 * shot_local_estimator.h
 *
 *  Created on: Mar 24, 2012
 *      Author: aitor
 */

#ifndef FAAT_PCL_REC_FRAMEWORK_OPENCV_SIFT_ESTIMATOR_H_
#define FAAT_PCL_REC_FRAMEWORK_OPENCV_SIFT_ESTIMATOR_H_

#include "local_estimator.h"
#include "faat_3d_rec_framework_defines.h"
#include <pcl/io/pcd_io.h>
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <v4r/ORUtils/pcl_opencv.h>

namespace faat_pcl
{
  namespace rec_3d_framework
  {

    template<typename PointInT, typename FeatureT>
      class FAAT_3D_FRAMEWORK_API OpenCVSIFTLocalEstimation : public LocalEstimator<PointInT, FeatureT>
      {

        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
        typedef typename pcl::PointCloud<FeatureT>::Ptr FeatureTPtr;

        using LocalEstimator<PointInT, FeatureT>::support_radius_;
        using LocalEstimator<PointInT, FeatureT>::normal_estimator_;
        using LocalEstimator<PointInT, FeatureT>::keypoint_extractor_;
          using LocalEstimator<PointInT, FeatureT>::adaptative_MLS_;
          using LocalEstimator<PointInT, FeatureT>::keypoint_indices_;
        pcl::PointIndices indices_;
        //cv::Ptr<cv::FeatureDetector> detectorPtr_;
        //cv::Ptr<cv::DescriptorExtractor> descriptorPtr_;
        cv::Ptr<cv::SIFT> sift_;

      public:

        size_t getFeatureType() const
        {
            return SIFT;
        }

        OpenCVSIFTLocalEstimation ()
        {
          const double threshold = 0.03;
          const double edge_threshold = 10.0;

          //detectorPtr_ = new cv::SiftFeatureDetector(threshold, edge_threshold);
          //descriptorPtr_  = cv::DescriptorExtractor::create("SIFT");
          sift_ = new cv::SIFT(0, 3, threshold, edge_threshold);
        }

        bool
        estimate (const PointInTPtr & in, PointInTPtr & processed, PointInTPtr & keypoints, FeatureTPtr & signatures)
        {

          keypoint_indices_.indices.clear();
          if(indices_.indices.size() == 0)
          {
            indices_.indices.resize(in->points.size());
            for(size_t i=0; i < indices_.indices.size(); i++)
            {
              indices_.indices[i] = i;
            }
          }

          processed.reset(new pcl::PointCloud<PointInT>);
          keypoints.reset(new pcl::PointCloud<PointInT>);

          pcl::PointCloud<int> mask_cloud;
          mask_cloud.width = in->width;
          mask_cloud.height = in->height;
          mask_cloud.points.resize(in->width * in->height);
          for(size_t i=0; i < mask_cloud.points.size(); i++)
            mask_cloud.points[i] = 0;

          for(size_t i=0; i < indices_.indices.size(); i++)
            mask_cloud.points[indices_.indices[i]] = 1;

          cv::Mat_ < cv::Vec3b > colorImage;
          PCLOpenCV::ConvertPCLCloud2Image<PointInT> (in, colorImage);
          cv::Mat grayImage;
          cv::cvtColor (colorImage, grayImage, CV_BGR2GRAY);

          cv::Mat descriptors;
          std::vector<cv::KeyPoint> ks;

          //detectorPtr_->detect(grayImage, ks);
          //descriptorPtr_->compute(grayImage, ks, descriptors);

          (*sift_)(grayImage, cv::Mat(), ks, descriptors, false);

          //use indices_ to check if the keypoints and feature should be saved
          //compute SIFT keypoints and SIFT features
          //backproject sift keypoints to 3D and save in keypoints
          //save signatures

          signatures->resize (ks.size ());
          signatures->width = static_cast<int> (ks.size ());
          signatures->height = 1;
          keypoints->points.resize(ks.size());
          int kept = 0;
          for(size_t i=0; i < ks.size(); i++)
          {
            int u,v;
            v = (int)(ks[i].pt.y+.5);
            u = (int)(ks[i].pt.x+.5);

            if(u >= mask_cloud.width)
            {
                PCL_ERROR("u bigger than width\n");
            }

            if(v >= mask_cloud.height)
            {
                PCL_ERROR("v bigger than heigth\n");
            }

            if(u >= 0 && v >= 0 && u < mask_cloud.width && v < mask_cloud.height && mask_cloud.at(u,v))
            {
              if(pcl_isfinite(in->at(u,v).z) && pcl_isfinite(in->at(u,v).x) && pcl_isfinite(in->at(u,v).y))
              {
                keypoints->points[kept] = in->at(u,v);
                keypoint_indices_.indices.push_back(v * in->width + u);
                assert((v * in->width + u) < (in->points.size()));
                for (int k = 0; k < 128; k++)
                  signatures->points[kept].histogram[k] = descriptors.at<float>(i,k);
                kept++;
              }
            }
          }

          signatures->width = kept;
          signatures->resize(kept);
          keypoints->points.resize(kept);
          pcl::copyPointCloud(*in, indices_, *processed);
          std::cout << "Number of SIFT features:" << kept << std::endl;
          indices_.indices.clear();

          return true;
        }

        void
        setIndices (const pcl::PointIndices & p_indices)
        {
          indices_ = p_indices;
        }

        void
        setIndices(const std::vector<int> & p_indices)
        {
          indices_.indices = p_indices;
        }

        bool acceptsIndices() const
        {
          return true;
        }

      private:

      };
  }
}

#endif /* REC_FRAMEWORK_SHOT_LOCAL_ESTIMATOR_H_ */
